#include <bits/stdc++.h>
using namespace std;

#define INF INT_MAX
#define MAX_CITIES 2000
#define MAX_EDGES 1000

struct Edge {
    int city;
    int weight;
};

struct GraphNode {
    vector<Edge> edges;
};

GraphNode graph[MAX_CITIES];
int n;

struct CityLocation {
    int x, y;
};
CityLocation cityLocations[MAX_CITIES];

struct AStarNode {
    int f_cost;  // f(x) = g(x) + h(x)
    int g_cost;  // actual distance from start
    int city;
};

struct PriorityQueue {
    vector<AStarNode> nodes;
    
    void push(int f_cost, int g_cost, int city) {
        nodes.push_back({f_cost, g_cost, city});
    }

    AStarNode pop() {
        if(nodes.empty()) return {INF, INF, -1};
        auto min_it = min_element(nodes.begin(), nodes.end(), [](AStarNode &a, AStarNode &b){
            return a.f_cost < b.f_cost;
        });
        AStarNode minNode = *min_it;
        nodes.erase(min_it);
        return minNode;
    }

    bool empty() { return nodes.empty(); }
};

enum Color { RED, BLACK };
struct RBTNode {
    int node1, node2;
    Color color;
    RBTNode *left, *right, *parent;
    RBTNode(int n1, int n2) : node1(n1), node2(n2), color(RED), left(nullptr), right(nullptr), parent(nullptr) {}
};
RBTNode *rbt_root = nullptr;

// ------------------- RBT Functions -------------------

void left_rotate(RBTNode *&root, RBTNode *x) {
    RBTNode *y = x->right;
    x->right = y->left;
    if(y->left) y->left->parent = x;
    y->parent = x->parent;
    if(!x->parent) root = y;
    else if(x == x->parent->left) x->parent->left = y;
    else x->parent->right = y;
    y->left = x;
    x->parent = y;
}

void right_rotate(RBTNode *&root, RBTNode *y) {
    RBTNode *x = y->left;
    y->left = x->right;
    if(x->right) x->right->parent = y;
    x->parent = y->parent;
    if(!y->parent) root = x;
    else if(y == y->parent->right) y->parent->right = x;
    else y->parent->left = x;
    x->right = y;
    y->parent = x;
}

void fix_insert(RBTNode *&root, RBTNode *z) {
    while(z->parent && z->parent->color == RED) {
        if(z->parent == z->parent->parent->left) {
            RBTNode *y = z->parent->parent->right;
            if(y && y->color == RED) {
                z->parent->color = BLACK;
                y->color = BLACK;
                z->parent->parent->color = RED;
                z = z->parent->parent;
            } else {
                if(z == z->parent->right) {
                    z = z->parent;
                    left_rotate(root, z);
                }
                z->parent->color = BLACK;
                z->parent->parent->color = RED;
                right_rotate(root, z->parent->parent);
            }
        } else {
            RBTNode *y = z->parent->parent->left;
            if(y && y->color == RED) {
                z->parent->color = BLACK;
                y->color = BLACK;
                z->parent->parent->color = RED;
                z = z->parent->parent;
            } else {
                if(z == z->parent->left) {
                    z = z->parent;
                    right_rotate(root, z);
                }
                z->parent->color = BLACK;
                z->parent->parent->color = RED;
                left_rotate(root, z->parent->parent);
            }
        }
    }
    root->color = BLACK;
}

void rbt_insert(RBTNode *&root, int node1, int node2) {
    RBTNode *z = new RBTNode(node1, node2);
    RBTNode *y = nullptr;
    RBTNode *x = root;
    while(x) {
        y = x;
        if(z->node1 < x->node1 || (z->node1 == x->node1 && z->node2 < x->node2)) x = x->left;
        else x = x->right;
    }
    z->parent = y;
    if(!y) root = z;
    else if(z->node1 < y->node1 || (z->node1 == y->node1 && z->node2 < y->node2)) y->left = z;
    else y->right = z;
    fix_insert(root, z);
}

// ------------------- Heuristic & A* -------------------

int heuristic(int city1, int city2) {
    int dx = cityLocations[city1].x - cityLocations[city2].x;
    int dy = cityLocations[city1].y - cityLocations[city2].y;
    return (int)sqrt(dx*dx + dy*dy);
}

void a_star(int source, int target, vector<int> &dist, vector<int> &prev) {
    dist.assign(n, INF);
    prev.assign(n, -1);
    dist[source] = 0;

    PriorityQueue pq;
    pq.push(heuristic(source,target), 0, source);

    while(!pq.empty()) {
        AStarNode current = pq.pop();
        int u = current.city;
        if(u == target) return;

        for(auto &e : graph[u].edges) {
            int v = e.city;
            int tentative = dist[u] + e.weight;
            if(tentative < dist[v]) {
                dist[v] = tentative;
                prev[v] = u;
                pq.push(tentative + heuristic(v,target), tentative, v);
            }
        }
    }
}

// ------------------- Graph I/O -------------------

int graph_input(const string &filename) {
    ifstream file(filename);
    if(!file.is_open()) {
        cerr << "Error opening file " << filename << endl;
        return -1;
    }
    for(int i=0;i<MAX_CITIES;i++) graph[i].edges.clear();

    int node1, node2, weight, max_city=-1;
    char comma;
    while(file >> node1 >> comma >> node2 >> comma >> weight) {
        if(node1>=MAX_CITIES || node2>=MAX_CITIES) continue;
        graph[node1].edges.push_back({node2,weight});
        rbt_insert(rbt_root,node1,node2);
        max_city = max(max_city, max(node1,node2));
    }
    return max_city;
}

void print_path(int start, int end, const vector<int> &prev) {
    if(prev[end]!=-1 && end!=start) print_path(start, prev[end], prev);
    if(end!=start) cout << " -> " << end;
}

void remove_edge(int node1, int node2) {
    auto &edges = graph[node1].edges;
    edges.erase(remove_if(edges.begin(), edges.end(), [&](Edge &e){ return e.city==node2; }), edges.end());
}

// ------------------- Randomization -------------------

void randomize_edge_weights() {
    for(int i=0;i<MAX_CITIES;i++)
        for(auto &e : graph[i].edges)
            e.weight = rand()%100 +1;
}

// ------------------- TSP A*-based -------------------

void tsp_a_star_based(int source, vector<int> &destinations, const string &filename) {
    int total_cost = 0;
    vector<bool> visited(MAX_CITIES,false);
    int current_city = source;
    visited[source]=true;
    cout << "Path: " << current_city;

    bool first_selection_done=false;
    vector<int> dist, prev;

    for(size_t count=0; count<destinations.size(); count++) {
        int nearest=-1, min_dist=INF;
        for(int dest: destinations) {
            if(!visited[dest]) {
                a_star(current_city,dest,dist,prev);
                if(dist[dest]<min_dist) {
                    min_dist=dist[dest];
                    nearest=dest;
                }
            }
        }
        if(nearest==-1){ cout << "\nError: No unvisited destinations\n"; return; }
        if(current_city!=nearest) print_path(current_city,nearest,prev);
        total_cost += min_dist;
        visited[nearest]=true;

        if(!first_selection_done && destinations.size()>1){
            randomize_edge_weights();
            first_selection_done=true;
        }

        current_city = nearest;
    }

    cout << "\nTotal minimal path cost: " << total_cost << endl;
}

// ------------------- Main -------------------

int main() {
    srand(time(0));

    string filename="graph_data.csv";
    n = graph_input(filename)+1;

    int num_excluded;
    cout << "Enter number of edges to exclude: ";
    cin >> num_excluded;
    for(int i=0;i<num_excluded;i++){
        int node1,node2;
        cout << "Enter edge to exclude (node1,node2): ";
        char comma;
        cin >> node1 >> comma >> node2;
        remove_edge(node1,node2);
        // RBT delete function can be added similarly
    }

    int source;
    cout << "Enter source city (0 to " << n-1 << "): ";
    cin >> source;

    int num_destinations;
    cout << "Enter number of destinations: ";
    cin >> num_destinations;
    vector<int> destinations(num_destinations);
    for(int i=0;i<num_destinations;i++){
        cout << "Destination " << i+1 << ": ";
        cin >> destinations[i];
    }

    tsp_a_star_based(source,destinations,filename);
    return 0;
}
