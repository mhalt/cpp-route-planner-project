#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = & m_Model.FindClosestNode(start_x, start_y);
    end_node = & m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	// Use the distance to the end_node for the h value.
	// Node objects have a distance method to determine the distance to another node.
    return node->distance(*end_node);
}

// The AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	try {
		// Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
		current_node->FindNeighbors();
		
		// For each node in current_node.neighbors, set the parent, the h_value, the g_value.
		for (RouteModel::Node *node : current_node->neighbors) {
			// We only want to modify unvisited nodes
			if(node->visited == false) {
				node->parent = current_node;
				// Use CalculateHValue below to implement the h-Value calculation.
				node->h_value = CalculateHValue(node);
				node->g_value = current_node->g_value + current_node->distance(*node);
				// Add the neighbor to open_list and set the node's visited attribute to true.
				open_list.emplace_back(node);
				node->visited = true;
			}
		}
	} catch (std::exception& e) {
		std::cout << "An exception occurred. Exception: " << e.what() << '\n';
	}
}

// The NextNode method sorts the open list and returns the next node.
RouteModel::Node *RoutePlanner::NextNode() {
	// Sort the open_list according to the sum of the h value and g value.
	// Declare a comparison lambda to satisfy requirements of std::sort
	std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *node_first, RouteModel::Node *node_second){
		auto node_first_sum = node_first->h_value + node_first->g_value;
		auto node_second_sum = node_second->h_value + node_second->g_value;
		// Return ​true if the first nodes' sum is less than the second
		return node_first_sum < node_second_sum;
	});
	
	// Create a pointer to the node in the list with the lowest sum.
	// std::sort Sorts the elements in the range [first, last) in non-descending order. 
	RouteModel::Node* next_node = open_list.front();
	// Remove that node from the open_list.
	open_list.erase(open_list.begin());
	// Return the pointer.
		
	return next_node;
}

/** 
	The ConstructFinalPath method returns the final path found from the A* search.
	This method takes the current (final) node as an argument and iteratively follows the 
	chain of parents of nodes until the starting node is found.
**/
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    // Create path_found vector
	std::vector<RouteModel::Node> path_found;
    RouteModel::Node *next_node_in_chain = current_node;

	while (next_node_in_chain != start_node) {
		path_found.push_back(*next_node_in_chain);
		// For each node in the chain, add the distance from the node to its parent to the distance variable.
		distance += next_node_in_chain->distance(*next_node_in_chain->parent);
		// The next node in the chain becomes the parent of the previous node	
		next_node_in_chain = next_node_in_chain->parent;
	}

	path_found.push_back(*next_node_in_chain);
	// Multiply the distance by the scale of the map to get meters.
	distance *= m_Model.MetricScale(); 
	// The returned vector should be in the correct order: std::reverse 
	// the start node should be the first element of the vector, the end node should be the last element.
	std::reverse(path_found.begin(), path_found.end());
	
    return path_found;
}

// Method to implement the A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    start_node->visited = true;

	while (current_node != end_node) {
		// Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
		AddNeighbors(current_node);
		// Use the NextNode() method to sort the open_list and return the next node.
		current_node = NextNode();
	}

	// When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
	// Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
	m_Model.path = ConstructFinalPath(end_node);

}