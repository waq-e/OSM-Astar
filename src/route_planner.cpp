#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
	// Convert inputs to percentage:
	start_x *= 0.01;
	start_y *= 0.01;
	end_x *= 0.01;
	end_y *= 0.01;

	// TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
	start_node = &(m_Model.FindClosestNode(start_x, start_y));
	end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	//float distanceToEnd = node->distance(*end_node);
	//return distanceToEnd;
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
	for (auto neighbor: current_node->neighbors) {
		neighbor->parent = current_node;
		neighbor->h_value = CalculateHValue(neighbor);
		// G_value is the sum of two values
		//		1- distance from parent of the node to start and this is simply represented by (current_node->g_value)
		//		2- distance from the current_node to the node and this could be gotten from this formula
		neighbor->g_value = current_node->g_value;
		neighbor->g_value += current_node->distance(*neighbor);
		neighbor->visited = true;
		open_list.emplace_back(neighbor);
	}
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
	// sort the open list by f-value (g+h)
	std::sort(begin(open_list), end(open_list),
	          [](RouteModel::Node *p1, RouteModel::Node *p2) {
		          return (((*p1).g_value + (*p1).h_value)) > (((*p2).g_value + (*p2).h_value));
	          });
	/*******************************************************************
	 * The above sort (using lambda) was adapted from the examples
	 * given by Kate Gregory in the course:
	 * "Beautiful C++: STL Algorithms."
	 * https://www.pluralsight.com/courses/beautiful-cplusplus-stl-algorithms
	 *
	 * - Rick Pendrick
	 *******************************************************************/

	auto nodeWithLowestFvalue = open_list.back();
	open_list.pop_back();

	return nodeWithLowestFvalue;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
	// Create path_found vector
	distance = 0.0f;
	std::vector<RouteModel::Node> path_found;

	// TODO: Implement your solution here.
	while (current_node != start_node) {
		auto distanceFromChildToParent = current_node->parent;
		distance += current_node->distance(*distanceFromChildToParent);
		path_found.push_back(*current_node);
		current_node = current_node->parent;
	}
	// start_node remains before reversing..
	path_found.push_back(*current_node);

	std::reverse(path_found.begin(),path_found.end());
	distance *= static_cast<float>(m_Model.MetricScale()); // Multiply the distance by the scale of the map to get meters.
	return path_found;
}


// TODO 7: Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
	open_list.push_back(start_node);
	start_node->visited = true;

	while (!open_list.empty()) {
		RouteModel::Node *current_node = NextNode();

		AddNeighbors(current_node);
		if (current_node == end_node) {
			m_Model.path = ConstructFinalPath(current_node);
		}
	} //end-while
}