#include<iostream>
#include<vector>
#include<algorithm>
#include<math.h>
#include<random>
int rowsize, colsize; // Grid size. These need to be accessed by the Astar_node class, hence global for now

class Astar_node{
public:
    int x;
    int y;
    float value;
    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    float f = 0;
    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    float g = 0;
    // Heuristic calculated value
    float h = 0;
    std::vector<std::vector<int>> neighbours;
    std::vector<std::vector<int>> previous;

    Astar_node(){
        value = 0;
    }

    void add_neighbours(std::vector<std::vector<Astar_node>>& grid_astar){
        // adding neighbours
        if(x < rowsize - 1){ 
            if (grid_astar[x + 1][y].value != 1) {  // Check for obstacle
               neighbours.push_back({x + 1, y});
            } 
        }
        if(y < colsize - 1){
            if (grid_astar[x][y+1].value != 1) {  // Check for obstacle
               neighbours.push_back({x, y+1});
            }
        }
        if(x > 0){ 
            if (grid_astar[x - 1][y].value != 1) {  // Check for obstacle
               neighbours.push_back({x - 1, y});
            }
        }
        if(y > 0){
            if (grid_astar[x][y-1].value != 1) {  // Check for obstacle
               neighbours.push_back({x, y - 1});
            }
        }
        // Adding diagonal elements
        if(x > 0 && y > 0){
            if (grid_astar[x - 1][y - 1].value != 1) {  // Check for obstacle
               neighbours.push_back({x - 1, y - 1});
            }
        }
        if(x < (rowsize - 1) && y < (colsize - 1)){
            if (grid_astar[x + 1][y + 1].value != 1) {  // Check for obstacle
               neighbours.push_back({x + 1, y + 1});
            }
        }
        if(x > 0 && y < (colsize - 1)){
            if (grid_astar[x - 1][y + 1].value != 1) {  // Check for obstacle
               neighbours.push_back({x - 1, y + 1});
            }
        }
        if(x < (rowsize - 1) && y > 0){
            if (grid_astar[x + 1][y - 1].value != 1) {  // Check for obstacle
               neighbours.push_back({x + 1, y - 1});
            }
        }
    }
};

float heuristic_function(Astar_node a, Astar_node b){
    float d1 = a.x - b.x;
    float d2 = a.y - b.y;

    float euclidean_distance = sqrt(pow(d1,2) + pow(d2,2));
    return euclidean_distance;
}

std::vector<std::vector<int>> find_path(std::vector<std::vector<Astar_node>>& grid_astar, std::vector<int> start, std::vector<int> goal) {

    std::vector<std::vector<int>> path;
    std::vector<std::vector<int>> open_set;
    std::vector<std::vector<int>> closed_set;
    // Add start point to open set
    open_set.push_back(start);

    while (!open_set.empty()) {
        int lowest_index = 0;
        // Find node with lowest f value
        for (int i = 0; i < open_set.size(); ++i) {
            if (grid_astar[open_set[i][0]][open_set[i][1]].f < grid_astar[open_set[lowest_index][0]][open_set[lowest_index][1]].f) {
                lowest_index = i;
            }
        }

        std::vector<int> current = open_set[lowest_index];
        open_set.erase(open_set.begin() + lowest_index); // erasing the element with lowest f-score from the open_set
        closed_set.push_back(current);

        if(current == goal) {
            std::vector<int> temp = {current[0], current[1]}; // current element
            while (grid_astar[temp[0]][temp[1]].previous.size() != 0) {
                path.push_back(temp);
                temp = grid_astar[temp[0]][temp[1]].previous[0];
            }
            if(temp[0] == 0 && temp[1] == 0){ // If temp = initial position. CHANGE IF INITIAL POSITION != 0
                path.push_back(temp);
            }
            for(int i = 0; i < path.size(); i++){
                // clearing previous values since they hinder the next function call.
                grid_astar[path[i][0]][path[i][1]].previous.clear();
            }
            break;
        }

        std::vector<std::vector<int>> current_neighbours = grid_astar[current[0]][current[1]].neighbours;

        for (auto& neighbour : current_neighbours) {

            auto& neighbour_node = grid_astar[neighbour[0]][neighbour[1]];

            if(neighbour_node.value == 0){
            // finding if the neighbours of the current grid element are in 
            // the closed_set or the open_set for evaluation
                auto it = std::find(closed_set.begin(), closed_set.end(), neighbour);
                auto it2 = std::find(open_set.begin(), open_set.end(), neighbour); 
                // Check if the neighbor is in the closed set
                if (it != closed_set.end()) {
                    continue; // Skip this neighbor if it's already evaluated or is an obstacle
                }
                // Calculate tentative g score
                float tentative_g = grid_astar[current[0]][current[1]].g + 1;

                // Check if the neighbor is in the open set
                if (it2 == open_set.end()) {
                    open_set.push_back(neighbour); // Add the neighbor to the open set
                } else if (tentative_g >= neighbour_node.g) {
                    continue; // Skip this neighbor since a better path already exists
                }

                // Update neighbor's scores and previous vector
                neighbour_node.g = tentative_g;
                neighbour_node.h = heuristic_function(neighbour_node, grid_astar[goal.at(0)][goal.at(1)]);
                neighbour_node.f = neighbour_node.g + neighbour_node.h;
                neighbour_node.previous = {current};
            }
            else continue;
        }
        if(open_set.empty()){
            path.push_back({0,0});
            std::cout<<"Path not found between "<<start.at(0)<<", "<<start.at(1)<<" and "<<goal.at(0)<<", "<<goal.at(1)<<". Try different obstacle placements."<<std::endl;
        }
    }
    return path;
}

bool is_far_from_obstacles(int x, int y, const std::vector<std::vector<int>>& waypoints) {
    const int min_distance_squared = 4; // Adjust as needed
    for (const auto& waypoint : waypoints) {
        int dx = x - waypoint[0];
        int dy = y - waypoint[1];
        if (dx * dx + dy * dy < min_distance_squared) {
            return false;
        }
    }
    // if(waypoints.back().at(0) == x || waypoints.back().at(1) == y){
    //     if(waypoints.back().at(0) < x)
    //     {
            
    //     }
    // }
    // if(grid_astar[i][j].value == 1){
    //     return false;
    // }
    return true;
}

int main(int argc, char **argv)
{
    std::cout<<"Enter grid size in row and column form: "<<std::endl;
    std::cin>>rowsize>>colsize;
    // Astar_node grid_astar[rowsize][colsize];
    std::vector<std::vector<Astar_node>> grid_vector(rowsize, std::vector<Astar_node>(colsize));

    for (int i = 0; i < rowsize; ++i) {
        for (int j = 0; j < colsize; ++j) {
            grid_vector[i][j].x = i;
            grid_vector[i][j].y = j; // Storing each gridpoint's x and y coordinates
            grid_vector[i][j].add_neighbours(grid_vector); // adding neighbours for each grid element

        }
    }
    // obstacles input
    int n_obstacles;
    std::cout<<"Enter number of obstacles to be considered: ";
    std::cin>>n_obstacles;
    
    if(n_obstacles > 0){
        int obstacle_positions[n_obstacles][3];

        for(int i=0; i<n_obstacles; i++){

            std::cout<<"Enter obstacle position "<<i+1<<" in spaced format: ";
            std::cin>>obstacle_positions[i][0]>>obstacle_positions[i][1];
            std::cout<<"Enter obstacle diameter "<<i+1<<" in spaced format: ";
            std::cin>>obstacle_positions[i][2];

            // handling illegal obstacle position cases
            if(obstacle_positions[i][0] > rowsize || obstacle_positions[i][0] < 0 || obstacle_positions[i][1]>colsize || obstacle_positions[i][1]<0){

                n_obstacles -= 1;
                i-=1;
                std::cout<<"The obstacle position is out of bounds. Ignoring...";

            }
            if(obstacle_positions[i][0] == obstacle_positions[i-1][0] && obstacle_positions[i][1] == obstacle_positions[i-1][1]){
            
                n_obstacles -= 1;
                i-=1;
                std::cout<<"Duplicate obstacle position. Ignoring...";
            
            }
            std::cout<<std::endl;
        }

        for(int i = 0; i < n_obstacles; i++){
            std::cout<<obstacle_positions[i][0]<<" "<<obstacle_positions[i][1]<<" "<<obstacle_positions[i][2]<<std::endl<<std::endl;
        }

        int k = n_obstacles-1;
        int row_modif, col_modif;
        while(k>=0){
            
            // Placing the obstacles on the grid
            int d = obstacle_positions[k][2];
            // centre of the obstacle
            int x = obstacle_positions[k][0];
            int y = obstacle_positions[k][1];
            // grid[x][y] = 1;
            grid_vector[x][y].value = 1;

            // radius of the obstacle

            int lim = (d%2 == 0 ? d/2 : (d/2)+1);
            if(d/2==0) lim = 0;
            
            for(int l = -lim; l <= lim; l++){
                for(int m = -lim; m <= lim; m++){
                    
                    row_modif = x+m;
                    col_modif = y+l;

                    if(row_modif >= rowsize || row_modif < 0 || col_modif >= colsize || col_modif < 0){
                        continue; // obstacle out of bounds
                    }
                    else{
                        // grid[row_modif][col_modif] = 1; // obstacle's bound coordinates are changed
                        grid_vector[row_modif][col_modif].value = 1;
                    }
                }
            }
            k--;
        }
    }
    std::cout<<"Grid with Obstacles:"<<std::endl;
    for(int i=0; i<rowsize; i++){
        for(int j=0; j<colsize; j++){
            std::cout<<" "<<grid_vector[i][j].value;
        }
        std::cout<<std::endl;
    }

    std::vector<std::vector<int>> waypoints;
    std::random_device rand;
    std::mt19937 gen(rand());
    waypoints.push_back({0,0}); // Initial position
    const int num_waypoints = 5; // Number of Waypoints
    while (waypoints.size() < num_waypoints) {
        std::uniform_int_distribution<>dis(0, rowsize - 1);
        std::uniform_int_distribution<>dis2(0, colsize - 1);
        int x = dis(gen);
        int y = dis2(gen);
        if (grid_vector[x][y].value == 0 && is_far_from_obstacles(x, y, waypoints)) {
            waypoints.push_back({x, y});
        }
    }

    std::vector<int> start = {0, 0};
    std::vector<int> goal = {rowsize - 1, colsize - 1};

    // Find path using A* algorithm
    std::vector<std::vector<int>> final_path;
    std::vector<std::vector<int>> no_path;
    no_path.push_back({0,0});
    
    for(int i = 0; i < (num_waypoints - 1); i++){
        std::vector<std::vector<int>> path = find_path(grid_vector, waypoints[i], waypoints[i+1]);
        if (path != no_path)
        {
            final_path.insert(final_path.begin(), path.begin(), path.end());
        }

    }

    std::cout << "Waypoints: "<< std::endl;
    for(int i = 0; i < waypoints.size(); i++){
        // for (int j = 0; j < waypoints[i].size(); j++) 
            waypoints[i][2] = abs(waypoints[i].at(0) - start.at(0)) + abs(waypoints[i].at(1) - start.at(1));
            waypoints[i][3] = sqrt(pow((waypoints[i].at(0) - start.at(0)),2) + pow((waypoints[i].at(1) - start.at(1)),2));
    }
    for (const auto& waypoint : waypoints) {
        
        std::cout << "Waypoint: (" << waypoint[0] << ", " << waypoint[1] << ")" << std::endl;
        std::cout<<"Manhattan Distance from Start: "<< waypoint[2] <<std::endl;
        std::cout<<"Euclidean Distance from Start: "<< waypoint[3] <<std::endl;
    }

    auto& grid_temp = grid_vector;
    std::cout << std::endl << "The Robot is marked with the letter R. "<<std::endl<<"final_path : " << std::endl;
    for (int i = final_path.size() - 1; i >= 0; i--) {
        std::cout << "("<< final_path[i][0] << ", " << final_path[i][1] << ")" << std::endl;
        grid_temp[final_path[i][0]][final_path[i][1]].value = 3;
    }
    std::cout<<std::endl<<"Grid with final_path: "<<std::endl<<std::endl;
    for(int i=0; i<rowsize; i++){
        for(int j=0; j<colsize; j++){
            if(grid_vector[i][j].value == 3){
                std::cout<<" R";
            }
            else std::cout<<" "<<grid_temp[i][j].value;
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;

    return 0;
}