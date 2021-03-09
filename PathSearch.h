#pragma once

#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/TileMap.h"
#include "../Framework/TileSystem/Tile.h"
#include "../PriorityQueue.h"

#include <vector>
#include <map>
#include <queue>
#include <string>
#include <cmath>
#include <iomanip>
#include <time.h>


namespace ufl_cap4053
{
	namespace searches
	{
		class PathSearch
		{
			class Edge {
				Tile* endpoint;
				float cost;
			public:
				Edge(Tile*, float);
				Tile* getEndpoint();
				float getCost();
			};
			
			class Vertex {
			public:
				Tile* vertex;
				std::vector<PathSearch::Edge*> edges;
				Vertex(Tile*);
			};

			class PlannerNode {
			public:
				Vertex* vertex;
				Vertex* parent;
				double gCost; //distance thus far
				double hCost; //distance to go to goal
				double fCost; //total path
				PlannerNode(Vertex*, Vertex*, double, double);
			};

			bool isFinished;

			Vertex* start = nullptr; Vertex* goal = nullptr;

			std::map<Tile*, Vertex*> nodes; //this is the search graph
			std::vector<Tile const*> solution; //this is the solution of ordered tiles the path planner found

			std::vector<PlannerNode*> openEnumeration;
			ufl_cap4053::PriorityQueue<PlannerNode*> open;
			//std::queue<PlannerNode*> open; //create a queue for vertices in the search graph
			std::map<Vertex*, PlannerNode*> visited;

		public:

			/*
			  ========== Search graph functions (for creating search graph from tile map) ==========
			*/
			DLLEXPORT void build_graph(TileMap* _tilemap); //TODO
			DLLEXPORT float calculateCost(TileMap* _tilemap, Tile* neighbor); //TODO
			DLLEXPORT double estDist(Vertex* current, Vertex* goal);
			DLLEXPORT std::vector<Tile*> tilesAdjacentTo(TileMap* _tilemap, int row, int col); //TODO
			DLLEXPORT bool isTraversableTile(TileMap* _tilemap, int row, int col); //TODO
			
			//compare function for priority queue;
			DLLEXPORT static bool compare(PlannerNode* const& lhs, PlannerNode* const& rhs);


			/*
			  ========== Search  functions (for searching tile map) ==========
			*/
			DLLEXPORT void build_solution(PlannerNode* current);

			/*
			  ========== Path Search functions (for running .exe) ==========
			*/
			DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
			DLLEXPORT ~PathSearch();
			DLLEXPORT void load(TileMap* _tileMap);
			DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
			DLLEXPORT void update(long timeslice);
			DLLEXPORT void shutdown();
			DLLEXPORT void unload();
			DLLEXPORT bool isDone() const;
			DLLEXPORT std::vector<Tile const*> const getSolution() const;
		};

	}
}  // close namespace ufl_cap4053::searches
