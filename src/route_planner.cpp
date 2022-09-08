#include "route_planner.h"
#include <algorithm>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    current_node->visited = true;
    for (RouteModel::Node *neighbor: current_node->neighbors)
    {
        neighbor->parent = current_node; // Set the parent
        neighbor->h_value = CalculateHValue(neighbor); // Set h_value - Use CalculateHValue
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); // Set g_value adding distance between neighboring nodes
        open_list.push_back(neighbor); // Add neighbor to open_list attribute
        neighbor->visited = true; // Set the node's visited attribute to true.
    }
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.

bool Compare(RouteModel::Node* first, RouteModel::Node* second){
    float score1 = first->g_value + first->h_value;
    float score2 = second->g_value + second->h_value;
    return score1 > score2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(this->open_list.begin(), this->open_list.end(), Compare); // Sort the open_list according to the sum of the h value and g value
    RouteModel::Node* next_node = open_list.back(); // Pointer to the lowest ranking h + g value in open_list
    return next_node; // Pointer to the lowest ranking g+h value node 
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->x != this->start_node->x && current_node->y != this->start_node->y) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end()); // Reverse vector

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    this->open_list.push_back(this->start_node);

    while (this->open_list.size() > 0) {

        current_node = this->NextNode();
        this->open_list.pop_back();
        if (current_node->distance(*RoutePlanner::end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}