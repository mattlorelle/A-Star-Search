#include "PathSearch.h"
#include "iostream"

namespace ufl_cap4053
{
	namespace searches
	{

		//Edge functions
		PathSearch::Edge::Edge(Tile* endpoint, float cost) {
			this->endpoint = endpoint;
			this->cost = cost;
		}
		float PathSearch::Edge::getCost() {
			return cost;
		}
		Tile* PathSearch::Edge::getEndpoint() {
			return endpoint;
		}


		//Vertex functions
		PathSearch::Vertex::Vertex(Tile* vertex) {
			this->vertex = vertex;
		}

		//PlannerNode functions
		PathSearch::PlannerNode::PlannerNode(Vertex* vertex, Vertex* parent, double gCost, double hCost) {
			this->vertex = vertex;
			this->parent = parent;
			this->gCost = gCost;
			this->hCost = 1.2 * hCost;
			fCost = this->gCost + (this->hCost);
		}

		//Additional PathSearch functions
		float PathSearch::calculateCost(TileMap* _tileMap, Tile* neighbor) {
			return (float)((2 * _tileMap->getTileRadius()) * (neighbor->getWeight()));
		}

		double PathSearch::estDist(Vertex* current, Vertex* goal) {
			return sqrt((pow(goal->vertex->getXCoordinate() - current->vertex->getXCoordinate(),2)) + (pow(goal->vertex->getYCoordinate() - current->vertex->getYCoordinate(),2)));
		}

		void PathSearch::build_graph(TileMap* _tilemap) {

			for (int i = 0; i < _tilemap->getRowCount(); i++) {
				for (int j = 0; j < _tilemap->getColumnCount(); j++) {

					if ((int)(_tilemap->getTile(i, j)->getWeight()) != 0) {
						Vertex* vertex = new Vertex(_tilemap->getTile(i, j));
						nodes.insert(std::pair<Tile*, Vertex*>(_tilemap->getTile(i, j), vertex));
					}
				}
			}

			std::map<Tile*, Vertex*>::iterator mapIter; //iterator to parse thru the map of tile, node pairs
			for (mapIter = nodes.begin(); mapIter != nodes.end(); mapIter++) {

				std::vector<Tile*> neighbors = tilesAdjacentTo(_tilemap, mapIter->first->getRow(), mapIter->first->getColumn()); //returns a list of tile * for all the neighboring tiles to the current tile in the tile,node map
				for (int i = 0; i < neighbors.size(); i++) {

					float cost = calculateCost(_tilemap, neighbors.at(i));
					mapIter->second->edges.push_back(new Edge(neighbors.at(i), cost));
				}
			}
		}

		std::vector<Tile*> PathSearch::tilesAdjacentTo(TileMap* _tilemap, int row, int col) {

			std::vector<Tile*> neighbors;

			if (row == 0) { //***if tile is in the top row of tilemap***


				if (col == 0) { //if tile is in top left corner of tilemap
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					return neighbors;
				}
				else if (col == (_tilemap->getColumnCount() - 1)) { //if tile is in top right corner
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row + 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col - 1)); //lower left neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					
					return neighbors;
				}
				else { //if tile is somewhere in the middle of the top row in the tilemap
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row + 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col - 1)); //lower left neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					
					return neighbors;
				}
			}
			else if (row == (_tilemap->getRowCount() - 1)) { //***if tile is in the bottom row of tilemap***


				if (row % 2 == 0) { //if last row is an even number
					if (col == 0) { //if tile is in top left corner of tilemap
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row, col + 1))
							neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
						return neighbors;
					}
					else if (col == (_tilemap->getColumnCount() - 1)) { //if tile is in top right corner
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row, col - 1))
							neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
						if (isTraversableTile(_tilemap, row - 1, col - 1))
							neighbors.push_back(_tilemap->getTile(row - 1, col - 1)); //upper left neighbor
						return neighbors;
					}
					else { //if tile is somewhere in the middle of the bottom row in the tilemap
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row, col + 1))
							neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
						if (isTraversableTile(_tilemap, row, col - 1))
							neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
						if (isTraversableTile(_tilemap, row - 1, col - 1))
							neighbors.push_back(_tilemap->getTile(row - 1, col - 1)); //upper left neighbor
						
						return neighbors;
					}
				}

				else { //if last row is an odd number
					if (col == 0) { //if tile is in top left corner of tilemap
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row - 1, col + 1))
							neighbors.push_back(_tilemap->getTile(row - 1, col + 1)); //upper right neighbor
						if (isTraversableTile(_tilemap, row, col + 1))
							neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
						return neighbors;
					}
					else if (col == (_tilemap->getColumnCount() - 1)) { //if tile is in top right corner
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row, col - 1))
							neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
						
						return neighbors;
					}
					else { //if the tile is in an odd last row and not on either side of the tilemap column wise
						if (isTraversableTile(_tilemap, row - 1, col))
							neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
						if (isTraversableTile(_tilemap, row - 1, col + 1))
							neighbors.push_back(_tilemap->getTile(row - 1, col + 1)); //upper right neighbor
						if (isTraversableTile(_tilemap, row, col + 1))
							neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
						if (isTraversableTile(_tilemap, row, col - 1))
							neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
						return neighbors;
					}
				}
			}
			else if (row % 2 == 0) { //***if tile is in an even row and not at top or bottom of tilemap***


				if (col == 0) { //if tile is in top left most column of tilemap
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					
					return neighbors;
				}
				else if (col == (_tilemap->getColumnCount() - 1)) { //if tile is in the right most column of tilemap
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row + 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col - 1)); //lower left neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					if (isTraversableTile(_tilemap, row - 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row - 1, col - 1)); //upper left neighbor
					return neighbors;
				}
				else { //if the tile is in an even row and not on either side of the tilemap column wise
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row + 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col - 1)); //lower left neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					if (isTraversableTile(_tilemap, row - 1, col - 1))
						neighbors.push_back(_tilemap->getTile(row - 1, col - 1)); //upper left neighbor
					
					
					return neighbors;
				}
			}
			else if (!(row % 2 == 0)) { //***if tile is in an odd row and not at top or bottom of tilemap***


				if (col == 0) { //if tile is in the top left most column of tilemap
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row - 1, col + 1))
						neighbors.push_back(_tilemap->getTile(row - 1, col + 1)); //upper right neighbor
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col + 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col + 1)); //lower right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					return neighbors;
				}
				else if (col == (_tilemap->getColumnCount() - 1)) { //if tile is in the right most column of tilemap
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					return neighbors;
				}
				else { //if the tile is in an add row and not on either side of the tilemap column wise
					if (isTraversableTile(_tilemap, row - 1, col))
						neighbors.push_back(_tilemap->getTile(row - 1, col)); //upper neighbor
					if (isTraversableTile(_tilemap, row - 1, col + 1))
						neighbors.push_back(_tilemap->getTile(row - 1, col + 1)); //upper right neighbor
					if (isTraversableTile(_tilemap, row, col + 1))
						neighbors.push_back(_tilemap->getTile(row, col + 1)); //right neighbor
					if (isTraversableTile(_tilemap, row + 1, col + 1))
						neighbors.push_back(_tilemap->getTile(row + 1, col + 1)); //lower right neighbor
					if (isTraversableTile(_tilemap, row + 1, col))
						neighbors.push_back(_tilemap->getTile(row + 1, col)); //lower neighbor
					if (isTraversableTile(_tilemap, row, col - 1))
						neighbors.push_back(_tilemap->getTile(row, col - 1)); //left neighbor
					return neighbors;
				}
			}

			return neighbors;
		}

		bool PathSearch::isTraversableTile(TileMap* _tilemap, int row, int col) {

			if (!(_tilemap->getTile(row, col)->getWeight() == 0))
				return true;
			else
				return false;
		}

		bool PathSearch::compare(PlannerNode* const& lhs, PlannerNode* const& rhs) {
			return lhs->fCost > rhs->fCost;
		}


		void PathSearch::build_solution(PlannerNode* goal) {

			solution.clear();

			PlannerNode* p = goal;

			while (p->vertex != start) {
				solution.push_back(p->vertex->vertex);
				p = visited[p->parent];
			}

			solution.push_back(p->vertex->vertex);
		}

		//Default PathSearch functions
		PathSearch::PathSearch() : open(compare) {
			isFinished = false;
		}

		PathSearch::~PathSearch() {

		}

		void PathSearch::load(TileMap* _tileMap) {

			nodes.clear(); //everytime a new tilemap is loaded into PathPlanner, the nodes map needs to be clear and set back to size 0
			build_graph(_tileMap); //builds a search graph of the current tile map
		}

		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {

			std::map<Tile*, Vertex*>::iterator it;
			for (it = nodes.begin(); it != nodes.end(); it++) {
				if (it->first->getRow() == startRow && it->first->getColumn() == startCol) {
					start = it->second;
				}
				if (it->first->getRow() == goalRow && it->first->getColumn() == goalCol) {
					goal = it->second;
				}
			}

			isFinished = false;
			
			PlannerNode* current = new PlannerNode(start, nullptr, 0, estDist(start, goal));
			open.push(current); //starting point has no parent
			visited.insert(std::pair<Vertex*, PlannerNode*>(start, open.front()));
		}

		void PathSearch::update(long timeslice) {
			
			if (timeslice == 0) {

				if (!open.empty()) { //while there is still a node in the list to be processed

					PlannerNode* current = open.front();
					open.pop();

					if (current->vertex == goal) {
						build_solution(current);
						isFinished = true;
					}

					for (int i = 0; i < current->vertex->edges.size(); i++) {

						double newCost = current->gCost + current->vertex->edges.at(i)->getCost();
						Vertex* successor = nodes[current->vertex->edges.at(i)->getEndpoint()];

						if (visited.count(successor) < 1) {
							PlannerNode* newNode = new PlannerNode(successor, current->vertex, newCost, estDist(current->vertex, goal));
							visited.insert(std::pair<Vertex*, PlannerNode*>(successor, newNode));
							open.push(newNode);
						}
						else {
							if (newCost < visited[successor]->gCost) {
								visited[successor]->gCost = newCost;
								visited[successor]->fCost = (visited[successor]->hCost) + newCost;
								visited[successor]->parent = current->vertex;
								if (open.front() != visited[successor])
									open.push(visited[successor]);
								else {
									open.remove(visited[successor]);
									open.push(visited[successor]);
								}
							}
						}
					}
				}
			}
			else {

				while (!open.empty()) { //while there is still a node in the list to be processed

					PlannerNode* current = open.front();
					open.pop();

					if (current->vertex == goal) {
						build_solution(current);
						isFinished = true;
					}

					for (int i = 0; i < current->vertex->edges.size(); i++) {

						double newCost = current->gCost + current->vertex->edges.at(i)->getCost();
						Vertex* successor = nodes[current->vertex->edges.at(i)->getEndpoint()];

						if (visited.count(successor) == 0) {
							PlannerNode* newNode = new PlannerNode(successor, current->vertex, newCost, estDist(current->vertex, goal));
							visited.insert(std::pair<Vertex*, PlannerNode*>(successor, newNode));
							open.push(newNode);
						}
						else {
							if (newCost < visited[successor]->gCost) {
								visited[successor]->gCost = newCost;
								visited[successor]->fCost = (visited[successor]->hCost) + newCost;
								visited[successor]->parent = current->vertex;
								if (open.front() != visited[successor])
									open.push(visited[successor]);
								else {
									open.remove(visited[successor]);
									open.push(visited[successor]);
								}
							}
						}
					}
				}
			}
		}

		void PathSearch::shutdown() {
			visited.clear();
			solution.clear();
		}

		void PathSearch::unload() {
			nodes.clear();
		}

		bool PathSearch::isDone() const {

			return isFinished;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const {

			return solution;
		}
	}
}  // close namespace ufl_cap4053::searches
