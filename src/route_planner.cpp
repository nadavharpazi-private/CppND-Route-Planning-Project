#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest nodes to the starting and ending coordinates.
    RouteModel::Node & closest_to_start = m_Model.FindClosestNode(start_x, start_y);
    RouteModel::Node & closest_to_end = m_Model.FindClosestNode(end_x, end_y);

    // Store those nodes in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &closest_to_start;
    this->end_node = &closest_to_end;
}

// CalculateHValue method: use the distance to the end_node for the h value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    if (node == nullptr) {
        std::cout << "RoutePlanner::CalculateHValue: got invalid input node (null)." << "\n";
        return -1;
    }

    auto h_distance = node->distance(*(this->end_node));
    return h_distance;
}

// AddNeighbors method: expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    for (RouteModel::Node *node: current_node->neighbors) {
        if (node->visited) {
            continue;
        }
        // For each unvisited node in current_node neighbors:
        // set the parent, the h_value, the g_value, and set visited attribute to true.
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        // finally, add the node to open_list
        open_list.push_back(node);
    }
}

// Compare the F values of two nodes.
// F values: the sum of the h value and g value.
bool Compare(const RouteModel::Node* node_a, const RouteModel::Node* node_b) {
    float f1 = node_a->h_value + node_a->g_value;
    float f2 = node_b->h_value + node_b->g_value;
    return f1 > f2;
}

// NextNode method: sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list
    sort(open_list.begin(), open_list.end(), Compare);

    // Create a pointer to the node in the list with the lowest sum.
    auto node = open_list.back();

    // Remove that node from the open_list.
    open_list.pop_back();

    // Return the pointer.
	return node;
}


// ConstructFinalPath method: return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // take the current (final) node and iteratively follow the chain
    // of parents of nodes until the starting node is found.
    while (current_node != nullptr) {
        // For each node in the chain, add the distance from the node to its parent.
        if (current_node->parent) {
            distance += current_node->distance(*current_node->parent);
        }
        // add the node to the path_found vector and then check if start node is reached
        path_found.push_back(*current_node);
        if ((current_node->x == start_node->x) &&
            (current_node->y == start_node->y)) {
            break;
        }
        current_node = current_node->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    // path_found order is from end to start.
    // reverse it to be in correct order: from start to end
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}

// the A* Search algorithm
void RoutePlanner::AStarSearch() {
    // start with start_node, set to true the visited flag and add it to open_list
    start_node->visited = true;
    RouteModel::Node *current_node = start_node;
    open_list.push_back(start_node);

    while (!open_list.empty())  {

        // add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);

        // return the next node (after sorting the open_list)
        current_node = NextNode();
        if ((current_node->x == end_node->x) &&
            (current_node->y == end_node->y)) {
            // search has reached the end_node, call ConstructFinalPath to return final path.
            std::vector<RouteModel::Node> path = ConstructFinalPath(current_node);
            // Store the final path in the m_Model.path attribute before the method exits.
            m_Model.path.assign(path.begin(), path.end());
            return;
        }
    }
    std::cout << "end_node not found, goal not reached." << "\n";
}
