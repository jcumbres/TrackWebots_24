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

/**
	\brief
	@author Javier Cumbres Tena, Telmo Clemente Serrano, Diego Gozalo García
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "Lidar3D.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
#include <locale>
#include <Eigen/Dense>
#include <timer/timer.h>
#include <qcustomplot/qcustomplot.h>
#include <boost/container_hash/hash.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void initialize();
	void compute();
	void new_mouse_coordinates(QPointF);

void reset_grid();

void emergency();
	void restore();
	int startup_check();

private:
	bool startup_check_flag;
	using GridPOS = std::optional<std::tuple<int, int>>;
	using LidarPOS = std::optional<std::tuple<float, float>>;
	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float DIMMENSION = 10000; // Dimmension(mm)
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1300;	// mm/s
		float MAX_ROT_SPEED = 2; // rad/s
		float TILE_SIZE = 100; // mm
		LidarPOS actual_target;
		std::vector<QPointF> actual_path;

		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
	};
	Params params;
	enum class CELL_STATE { EMPTY, OCCUPIED, UNKNOWN };

	typedef struct TCell
	{
		CELL_STATE state = CELL_STATE::UNKNOWN;
		int x, y;
		QGraphicsRectItem *rect;

		// Operador == para comparar dos objetos TCell
		bool operator==(const TCell& other) const {
			return (x == other.x && y == other.y && state == other.state);
		}

		// Operador < para permitir que TCell sea usado en std::map
		bool operator<(const TCell& other) const {
			if (x == other.x)
				return y < other.y;
			return x < other.x;
		}

		bool has_value() const {
			return (x >= 0 && x <= 100) && (y >= 0 && y <= 100);
		}

		std::pair<float, float> value() const {
			return {x, y};
		}

	} TCell;

	// lidar
	std::vector<Eigen::Vector2f> read_lidar_bpearl();

	// draw
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;

    // grid
    static constexpr int GRID_SIZE = 100;
    std::array<std::array<TCell, GRID_SIZE>, GRID_SIZE> grid;
	void changeState(auto &filtered_points);

    // Coordinates
	void transformToGRID();

    // Transformations
    std::optional<std::pair<int, int>> grid_to_lidar(float i, float j);
	std::optional<std::pair<float, float>> lidar_to_grid(float x, float y);
	LidarPOS lidar_to_grid_pos(GridPOS gp);
	GridPOS grid_to_lidar_pos(LidarPOS lp);
	QString cellStateToString(CELL_STATE state);


	//Dijkstra
	std::vector<TCell> get_neighbors(TCell& current, std::array<std::array<TCell, GRID_SIZE>, GRID_SIZE>& grid);
	std::vector<QPointF> dijkstra(GridPOS start, GridPOS target);
	std::vector<QPointF> dijkstra2(GridPOS start, GridPOS target);
	void draw_target(QPointF goal, bool erase = false);
	void draw_path(std::vector<QPointF> &path, QGraphicsScene *scene);
	void clean_near_data(GridPOS target);
	std::vector<QPointF> smooth_path(const std::vector<QPointF>& path);


	//Comunication
	RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target) override;
};

#endif
