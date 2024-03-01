#include<iostream>
#include<vector>
#include<algorithm>
#include<math.h>
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

        void add_neighbours(){
            // adding neighbours
            if(x < rowsize - 1){ 
                neighbours.push_back({x+1, y});   
            }
            if(y < colsize - 1){
                neighbours.push_back({x, y+1});
            }
            if(x > 0){ 
                neighbours.push_back({x-1, y}); 
            }
            if(y > 0){
                neighbours.push_back({x, y-1});
            }
            // Adding diagonal elements
            if(x > 0 && y > 0){
                neighbours.push_back({x-1, y-1});
            }
            if(x < (rowsize - 1) && y < (colsize - 1)){
                neighbours.push_back({x+1, y+1});
            }
            if(x > 0 && y < (colsize - 1)){
                neighbours.push_back({x-1, y+1});
            }
            if(x < (rowsize - 1) && y > 0){
                neighbours.push_back({x+1, y-1});
            }
        }
};

float heuristic_function(Astar_node a, Astar_node b){
    float d1 = a.x - b.x;
    float d2 = a.y - b.y;

    float euclidean_distance = sqrt(pow(d1,2) + pow(d2,2));
    float manhattan_distance = abs(d1) + abs(d2);

    return euclidean_distance;
}

int main(int argc, char **argv)
{

    std::cout<<"Enter grid size in row and column form: "<<std::endl;
    std::cin>>rowsize>>colsize;
    Astar_node grid_astar[rowsize][colsize];

    for(int i = 0; i < rowsize; i++){
        for(int j = 0; j < colsize; j++){
            grid_astar[i][j].x = i;
            grid_astar[i][j].y = j; // Storing each gridpoint's x and y coordinates
            grid_astar[i][j].add_neighbours(); // adding neighbours for each grid element
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

        int k = n_obstacles-1;
        int row_modif, col_modif;
        while(k>=0){
            
            // Placing the obstacles on the grid
            int d = obstacle_positions[k][2];
            // centre of the obstacle
            int x = obstacle_positions[k][0];
            int y = obstacle_positions[k][1];
            // grid[x][y] = 1;
            grid_astar[x][y].value = 1;

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
                        grid_astar[row_modif][col_modif].value = 1;
                    }
                }
            }
            k--;
        }
    }
    std::cout<<"Grid with obstacles: "<<std::endl;
    for(int i=0; i<rowsize; i++){
        for(int j=0; j<colsize; j++){
            std::cout<<" "<<grid_astar[i][j].value;
        }
        std::cout<<std::endl;
    }

    // ------------------------------------------
    // Obstacle Avoidance Logic - Using the A* Algorithm
    // ------------------------------------------

    std::vector<std::vector<int>> open_set; // stores a list of nodes remaining to be evaluated
    // start the algo with the initial position
    open_set.push_back({0,0});

    std::vector<std::vector<int>> closed_set; // stores a list of nodes that have finished being evaluated
    // int closed_set[rowsize][colsize] = {}; 
    // For node n, its 'previous' value is the node immediately preceding it on the 
    // cheapest path from the start to n currently known.
    std::vector<std::vector<int>> path;
    // Starting position = (0,0)
    int end_position[2] = {rowsize-1, colsize-1}; // end position - CAN BE CHANGED

    // A* algorithm loop goes here
    while (!open_set.empty()) { // while open_set is not empty
        // Go ahead
        int lowest_index = 0;
        for (int z = 0; z < open_set.size(); z++) {
            if (grid_astar[open_set[z][0]][open_set[z][1]].f < grid_astar[open_set[lowest_index][0]][open_set[lowest_index][1]].f) {
                // The grid point with the lowest 'f' score
                lowest_index = z;
            }
        }

        std::vector<int> current = open_set[lowest_index]; // setting the current coordinates as the point on the grid with the lowest f-score
        open_set.erase(open_set.begin() + lowest_index); // erasing the element with lowest f-score from the open_set
        closed_set.push_back(current); // add the lowest f-score index to closed set

        // Checking for the end of the path searching
        if (current[0] == end_position[0] && current[1] == end_position[1]) {
            std::vector<int> temp = {current[0], current[1]}; // current element
            while (grid_astar[temp[0]][temp[1]].previous.size() != 0) {
                path.push_back(temp);
                temp = grid_astar[temp[0]][temp[1]].previous[0];
            }
            path.push_back(temp);
            std::cout << "Path found." << std::endl;
            break;
        }

        std::vector<std::vector<int>> current_neighbours = grid_astar[current[0]][current[1]].neighbours; // neighbours of the current grid element

        for (auto& neighbour : current_neighbours) {
            // finding if the neighbours of the current grid element are in 
            // the closed_set or the open_set for evaluation
            auto it = std::find(closed_set.begin(), closed_set.end(), neighbour);
            auto it2 = std::find(open_set.begin(), open_set.end(), neighbour);
            auto& neighbour_node = grid_astar[neighbour[0]][neighbour[1]];

            // Check if the neighbor is in the closed set
            if (it != closed_set.end() || neighbour_node.value == 1) {
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
            neighbour_node.h = heuristic_function(neighbour_node, grid_astar[end_position[0]][end_position[1]]);
            neighbour_node.f = neighbour_node.g + neighbour_node.h;
            neighbour_node.previous = {current};
        }
        if(open_set.empty()){
            std::cout<<"Path not found. Try different obstacle placements."<<std::endl;
        }
    }

    auto& grid_temp = grid_astar;
    std::cout << "The Robot is marked with the letter R. "<<std::endl<<"Path : " << std::endl;
    for (int i = path.size() - 1; i >= 0; i--) {
        std::cout << "("<< path[i][0] << ", " << path[i][1] << ")" << std::endl;
        grid_temp[path[i][0]][path[i][1]].value = 3;
    }
    // Printing the grid with the obstacle-avoiding values
    std::cout<<std::endl<<"Grid with Path: "<<std::endl<<std::endl;
    for(int i=0; i<rowsize; i++){
        for(int j=0; j<colsize; j++){
            if(grid_temp[i][j].value == 3){
                std::cout<<" R";
            }
            else std::cout<<" "<<grid_temp[i][j].value;
        }
        std::cout<<std::endl;
    }

    return 0;
}
