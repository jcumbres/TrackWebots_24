/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/range.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

	return true;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		viewer->setStyleSheet("background-color: lightGray;");
		this->resize(800, 700);
		viewer->show();

		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(new_mouse_coordinates(QPointF)));

		transformToGRID();

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);

	}
}

void SpecificWorker::compute()
{
	reset_grid();
	//read bpearl (lower) lidar and draw
	auto ldata_bpearl = read_lidar_bpearl();
	if(ldata_bpearl.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
	draw_lidar(ldata_bpearl, &viewer->scene);
	if (params.actual_path.size() > 0 and params.actual_target.has_value())
	{
		draw_path(params.actual_path, &viewer->scene);
		auto &[x,y] = *params.actual_target;
		draw_target(QPointF{x,y});
		}
	changeState(ldata_bpearl);
}

void SpecificWorker::new_mouse_coordinates(QPointF goal)
{
	qDebug() << "New mouse coordinates: " << goal;
	draw_target(goal);
	const auto target= lidar_to_grid(goal.x(), goal.y());
	const GridPOS start {{GRID_SIZE/2, GRID_SIZE/2}};
	auto path = dijkstra(start,target);
	qDebug() << "Path size: " << path.size();

	draw_path(path, &viewer->scene);
}

//YOUR CODE HERE
void SpecificWorker::reset_grid()
{
	for (auto &fila : grid)
		for (auto &celda : fila)
		{
			celda.state = CELL_STATE::UNKNOWN;
			celda.rect->setBrush(QBrush(Qt::lightGray));
		}
}

std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}

/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::darkGreen);
	auto brush = QBrush(QColor(Qt::darkGreen));
	for(const auto &p : filtered_points)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
}

void SpecificWorker::transformToGRID()
{
	std::array<std::array<TCell, GRID_SIZE>, GRID_SIZE> _grid;
	int i = 0, j = 0;
	QPen pen_e(Qt::blue, 5);
	QBrush brush(Qt::lightGray);
	for (auto &fila :grid)
	{
		j = 0;
		for (auto &celda : fila)
		{
			celda.state = CELL_STATE::EMPTY;
			celda.x = i;
			celda.y = j;
			// Obtener la posición de la celda en el mundo
			auto result = grid_to_lidar(i, j);
			auto &[x,y] = *result;
			// Dibujar la celda
			celda.rect = viewer->scene.addRect(x,y, params.TILE_SIZE, params.TILE_SIZE, pen_e, brush);
			//celda.rect->setPos(celda.x, celda.y);
			j++;
		}
		i++;
	}
}

std::optional<std::pair<int, int>> SpecificWorker::grid_to_lidar(float i, float j)
{
	float x = params.DIMMENSION / GRID_SIZE * i - params.DIMMENSION / 2;
	float y = params.DIMMENSION / GRID_SIZE * j - params.DIMMENSION / 2;
	return std::make_pair(x, y);
}

std::optional<std::pair<float, float>> SpecificWorker::lidar_to_grid(float x, float y){
	int i = std::clamp(static_cast<int>((GRID_SIZE/params.DIMMENSION) * x + GRID_SIZE/2), 0, GRID_SIZE-1);
	int j = std::clamp(static_cast<int>((GRID_SIZE/params.DIMMENSION) * y + GRID_SIZE/2), 0, GRID_SIZE-1);
	if (i < 0 or j < 0 or i >= GRID_SIZE or j >= GRID_SIZE)
		return {};
	return std::make_pair(i, j);
}

SpecificWorker::LidarPOS SpecificWorker::lidar_to_grid_pos(GridPOS gp)
{
	auto &[i,j] = *gp;
	auto result = grid_to_lidar(i,j);
	auto &[x,y] = *result;
	return LidarPOS{{x,y}};
}

SpecificWorker::GridPOS SpecificWorker::grid_to_lidar_pos(LidarPOS lp)
{
	auto &[x, y] = *lp;
	auto result = lidar_to_grid(x, y);
	auto &[i, j] = *result;
	return {{i, j}};
}

QString SpecificWorker::cellStateToString(CELL_STATE state)
{
	switch (state) {
		case CELL_STATE::UNKNOWN: return "UNKNOWN";
		case CELL_STATE::OCCUPIED: return "OCCUPIED";
		case CELL_STATE::EMPTY: return "EMPTY";
		default: return "INVALID_STATE";
	}
}

void SpecificWorker::changeState(auto &filtered_points)
{
	QBrush brush_in(Qt::white);
	QBrush brush_out(Qt::red);
	for (const auto &point: filtered_points)
	{
		const float S = std::hypot(point.x(), point.y()) / params.TILE_SIZE;
		Eigen::Vector2f p = {0.f, 0.f};
		for (const auto o: iter::range(0.f, 1.f, 1/S))
		{
			p = point * o;
			 const auto result = lidar_to_grid(p.x(), p.y());
			if (result)
			{
				const auto &[i, j] = *result;
				grid[i][j].rect->setBrush(brush_in);
			}
		}
		if (const auto result = lidar_to_grid(p.x(), p.y()))
		{
			const auto &[i, j] = *result;
			grid[i][j].rect->setBrush(brush_out);
			std::vector<std::pair<int, int>> neighbours = {{-1,-1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
			for (auto n: neighbours) {
				int a = i + n.first;
				int b = j + n.second;
				if (!(a < 0 or b < 0 or a >= GRID_SIZE or b >= GRID_SIZE))
				{
					grid[a][b].rect->setBrush(brush_out);
					grid[a][b].state = CELL_STATE::OCCUPIED;
				}
			}
			grid[i][j].state = CELL_STATE::OCCUPIED;
		}
	}
}

void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
	//  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




std::vector<QPointF> SpecificWorker::dijkstra(GridPOS start, GridPOS target)
{
	 // Verificar que las posiciones de inicio y objetivo son válidas
    if (!start.has_value() || !target.has_value())
    {
        qWarning() << "Posición de inicio o objetivo no válida.";
        return {};
    }

    // Desempaquetar las posiciones
    auto [start_x, start_y] = start.value();
    auto [target_x, target_y] = target.value();

    // Direcciones de movimiento (arriba, abajo, izquierda, derecha)
    const std::vector<std::tuple<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    // Mapa de distancias mínimas desde el inicio
    std::unordered_map<std::tuple<int, int>, int, boost::hash<std::tuple<int, int>>> min_distance;

    // Mapa para reconstruir el camino
    std::unordered_map<std::tuple<int, int>, std::tuple<int, int>, boost::hash<std::tuple<int, int>>> previous;

    // Inicializar distancias
    for (int i = 0; i < GRID_SIZE ; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            std::tuple<int, int> pos = {i, j};
            if (grid[i][j].state == CELL_STATE::OCCUPIED or grid[i][j].state == CELL_STATE::UNKNOWN)
                min_distance[pos] = std::numeric_limits<int>::max(); // Obstáculo
            else
                min_distance[pos] = std::numeric_limits<int>::max();
        }
    }
    // Distancia al punto inicial es 0
    std::tuple<int, int> start_pos = {start_x, start_y};
    min_distance[start_pos] = 0;

    // Crear la cola de prioridad (min heap)
    std::priority_queue<std::pair<int, std::tuple<int, int>>,
                        std::vector<std::pair<int, std::tuple<int, int>>>,
                        std::greater<>> pq;
    pq.push({0, start_pos});
    // Algoritmo de Dijkstra
    while (!pq.empty())
    {
        auto [current_dist, current] = pq.top();
        pq.pop();

         // Si llegamos al objetivo, detener
        if (current == std::make_tuple(target_x, target_y))
            break;

        // Obtener coordenadas del nodo actual
        auto [cx, cy] = current;

        // Explorar vecinos
        for (const auto &[dx, dy] : directions)
        {
            int nx = cx + dx;
            int ny = cy + dy;

            // Verificar límites de la cuadrícula
            if (nx < 0 ||  ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
                continue;

            // Verificar si la celda es transitable
            if (grid[nx][ny].state == CELL_STATE::OCCUPIED)
                continue;

            std::tuple<int, int> neighbor = {nx, ny};
            int new_dist = current_dist + 1; // Coste uniforme de 1 para cada movimiento

            // Actualizar si se encuentra un camino más corto
            if (new_dist < min_distance[neighbor])
            {
                min_distance[neighbor] = new_dist;
                previous[neighbor] = current;
                pq.push({new_dist, neighbor});
            }
        }
    }

    // Reconstruir el camino desde el objetivo al inicio
    std::vector<std::tuple<int, int>> path;
    std::tuple<int, int> current = {target_x, target_y};

    while (current != start_pos)
    {
        path.push_back(current);
        if (previous.find(current) == previous.end())
        {
            qWarning() << "No se encontró un camino válido.";
            return {};
        }
        current = previous[current];
    }
    path.push_back(start_pos);
    std::reverse(path.begin(), path.end());




    // Pintar el camino en la cuadrícula
	std::vector<QPointF> path_points;
	path_points.reserve(path.size());
	for (const auto &[x, y] : path)
	{
		const auto result = grid_to_lidar(x, y);
		if (result)
		{
			const auto &[i, j] = *result;
			path_points.emplace_back(QPoint(i,j));
		}
	}

	return path_points;
}

std::vector<QPointF> SpecificWorker::dijkstra2(GridPOS start, GridPOS target)
{
    // Verificar que las posiciones de inicio y objetivo son válidas
    if (!start.has_value() || !target.has_value())
    {
        qWarning() << "Posición de inicio o objetivo no válida.";
        return {};
    }

    // Desempaquetar las posiciones
    auto [start_x, start_y] = start.value();
    auto [target_x, target_y] = target.value();

    // Direcciones de movimiento (arriba, abajo, izquierda, derecha)
    const std::vector<std::tuple<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    // Mapa de distancias mínimas desde el inicio
    std::unordered_map<std::tuple<int, int>, int, boost::hash<std::tuple<int, int>>> min_distance;

    // Mapa para reconstruir el camino
    std::unordered_map<std::tuple<int, int>, std::tuple<int, int>, boost::hash<std::tuple<int, int>>> previous;

    // Inicializar distancias
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            std::tuple<int, int> pos = {i, j};
            if (grid[i][j].state == CELL_STATE::OCCUPIED || grid[i][j].state == CELL_STATE::UNKNOWN)
                min_distance[pos] = std::numeric_limits<int>::max(); // Obstáculo
            else
                min_distance[pos] = std::numeric_limits<int>::max();
        }
    }

    // Distancia al punto inicial es 0
    std::tuple<int, int> start_pos = {start_x, start_y};
    min_distance[start_pos] = 0;

    // Crear la cola de prioridad (min heap)
    std::priority_queue<std::pair<int, std::tuple<int, int>>,
                        std::vector<std::pair<int, std::tuple<int, int>>>,
                        std::greater<>> pq;
    pq.push({0, start_pos});

    // Algoritmo de Dijkstra
    while (!pq.empty())
    {
        auto [current_dist, current] = pq.top();
        pq.pop();

        // Si llegamos al objetivo, detener
        if (current == std::make_tuple(target_x, target_y))
            break;

        // Obtener coordenadas del nodo actual
        auto [cx, cy] = current;

        // Explorar vecinos
        for (const auto &[dx, dy] : directions)
        {
            int nx = cx + dx;
            int ny = cy + dy;

            // Verificar límites de la cuadrícula
            if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
                continue;

            // Verificar si la celda es transitable y si sus vecinos en un rango 7x7 también lo son
            bool safe_to_move = true;
            for (int i = -3; i <= 3 && safe_to_move; ++i) // Expandimos el rango a -3, 3 para cubrir 7x7
            {
                for (int j = -3; j <= 3; ++j)
                {
                    int neighbor_x = nx + i;
                    int neighbor_y = ny + j;

                    if (neighbor_x < 0 || neighbor_y < 0 || neighbor_x >= GRID_SIZE || neighbor_y >= GRID_SIZE)
                        continue;

                    if (grid[neighbor_x][neighbor_y].state == CELL_STATE::OCCUPIED)
                    {
                        safe_to_move = false;
                        break;
                    }
                }
            }

            if (!safe_to_move)
                continue;

            std::tuple<int, int> neighbor = {nx, ny};
            int new_dist = current_dist + 1; // Coste uniforme de 1 para cada movimiento

            // Actualizar si se encuentra un camino más corto
            if (new_dist < min_distance[neighbor])
            {
                min_distance[neighbor] = new_dist;
                previous[neighbor] = current;
                pq.push({new_dist, neighbor});
            }
        }
    }

    // Reconstruir el camino desde el objetivo al inicio
    std::vector<std::tuple<int, int>> path;
    std::tuple<int, int> current = {target_x, target_y};

    while (current != start_pos)
    {
        path.push_back(current);
        if (previous.find(current) == previous.end())
        {
            qWarning() << "No se encontró un camino válido.";
            return {};
        }
        current = previous[current];
    }
    path.push_back(start_pos);
    std::reverse(path.begin(), path.end());

    // Pintar el camino en la cuadrícula
    std::vector<QPointF> path_points;
    path_points.reserve(path.size());
    for (const auto &[x, y] : path)
    {
        const auto result = grid_to_lidar(x, y);
        if (result)
        {
            const auto &[i, j] = *result;
            path_points.emplace_back(QPointF(i, j));
        }
    }

    return path_points;
}


void SpecificWorker::draw_target(QPointF goal, bool erase)
{
	static QGraphicsItem* item;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	if (item != nullptr) {
		viewer->scene.removeItem(item);
		delete item;
	}

	if (erase)
		return;

	item = viewer->scene.addRect(goal.x(), goal.y(), params.TILE_SIZE, params.TILE_SIZE, QPen(Qt::red), QBrush(Qt::red));
}

void SpecificWorker::draw_path(std::vector<QPointF> &path, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items) {
		scene->removeItem(i);
		delete i;
	}

	items.clear();
	const QBrush brush(Qt::blue);
	for (const auto &[x, y] : path) {
		auto i = scene->addEllipse(-50, -50, 100, 100, QPen (Qt::blue), brush);
		i->setPos(x,y);
		items.push_back(i);
	}
}

void SpecificWorker::clean_near_data(GridPOS target)
{
	const auto &[i, j] = *target;
	std::vector<std::pair<int, int>> neighbours = {{-1,-1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
	for (auto n: neighbours) {
		int a = i + n.first;
		int b = j + n.second;
		if (!(a < 0 or b < 0 or a >= GRID_SIZE or b >= GRID_SIZE))
		{
			grid[a][b].state = CELL_STATE::UNKNOWN;
		}
	}
	grid[i][j].state = CELL_STATE::UNKNOWN;
}

std::vector<QPointF> SpecificWorker::smooth_path(const std::vector<QPointF> &path)
{
	if (path.size() < 3)
		return path; // Si el camino tiene menos de 3 puntos, no se puede suavizar

	// Copiar el camino original para trabajar sobre él
	std::vector<QPointF> smoothed_path = path;

	// Parámetros de suavizado
	const float alpha = 0.1; // Peso para acercarse al promedio de los vecinos
	const float beta = 0.1;  // Peso para mantener cercanía al camino original
	const int iterations = 10; // Número de iteraciones para suavizar

	for (int it = 0; it < iterations; ++it)
	{
		for (size_t i = 1; i < smoothed_path.size() - 1; ++i)
		{
			// Puntos vecinos
			QPointF prev = smoothed_path[i - 1];
			QPointF next = smoothed_path[i + 1];

			// Punto original
			QPointF original = path[i];

			// Desplazamiento de vértice
			smoothed_path[i].setX(smoothed_path[i].x() + alpha * (prev.x() + next.x() - 2 * smoothed_path[i].x()) + beta * (original.x() - smoothed_path[i].x()));
			smoothed_path[i].setY(smoothed_path[i].y() + alpha * (prev.y() + next.y() - 2 * smoothed_path[i].y()) + beta * (original.y() - smoothed_path[i].y()));
		}
	}

	return smoothed_path;
}



RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
	LidarPOS _source {{source.x, source.y}};
	LidarPOS _target  {{target.x, target.y}};
	GridPOS goal = grid_to_lidar_pos(_target);
	params.actual_target = _source;
	GridPOS start = grid_to_lidar_pos(_source);
	clean_near_data(goal);
	std::vector<QPointF> path = dijkstra(start,goal);
	std::vector<QPointF> smoothed_path = smooth_path(path);
	qDebug() << "Tamaño del camino" << path.size();
	RoboCompGrid2D::Result result;
	result.path.reserve(smoothed_path.size()); // Reservar memoria para evitar realocaciones
	std::ranges::transform(smoothed_path, std::back_inserter(result.path), [](auto &p) {
							return RoboCompGrid2D::TPoint{static_cast<float>(p.x()), static_cast<float>(p.y()), 0.f};
	});
	params.actual_path.clear();
	params.actual_path = smoothed_path; // Realiza una copia completa
	qDebug() << "Tamaño del camino" << result.path.size();
	return result;
}


/**************************************/
// From the RoboCompGrid2D you can call this methods:
// this->grid2d_proxy->getPaths(...)

/**************************************/
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

