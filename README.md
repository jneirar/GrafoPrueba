# Algoritmos y Estructura de Datos
## Proyecto sobre Grafos
--------

## Integrantes
- Neira Riveros, Jorge Luis
- Berendson Villanueva, Sebastian
- Escalante Ccoyllo, Sebastian Amaru

----

El proyecto del curso consiste en implementar una estructura de datos de grafo y un file parser. La estructura debe soportar los métodos y algoritmos descritos a continuacion:  


## Graph data structure

* El grafo debe ser dinámico (inserciones. eliminaciones, búsquedas, ...)
* Se debe implementar los dos tipos de grafos: dirigidos y no-dirigidos.
* No considerar loops ni multi-arista. 


### Methods:
```cpp
bool insertVertex(string id, V data); // Creates a new vertex in the graph with some data and an ID

bool createEdge(string start, string end, E data); // Creates a new edge in the graph with some data

bool deleteVertex(string id); // Deletes a vertex in the graph

bool deleteEdge(string start, string end); // Deletes an edge in the graph, it is not possible to search by the edge value, since it can be repeated

E &operator()(string start, string end); // Gets the value of the edge from the start and end vertexes

float density() const; // Calculates the density of the graph

bool isDense(float threshold = 0.5) const; // Calculates the density of the graph, and determine if it is dense dependening on a threshold value

bool isConnected(); // Detect if the graph is connected

bool isStronglyConnected() throw(); // Detect if the graph is strongly connected (only for directed graphs)

bool empty(); // If the graph is empty

void clear(); // Clears the graph
```

### Algorithms (Part 1):
```cpp
//Given the graph
UndirectedGraph<char, int> graph;

//1- Generates a MST graph using the Kruskal approach (only for undirected graphs)
Kruskal<char, int> kruskal(&graph);
UndirectedGraph<char, int> result = kruskal.apply();//return a tree

//2- Generates a MST graph using the Prim approach (only for undirected graphs)
Prim<char, int> prim(&graph, id);
UndirectedGraph<char, int> result = prim.apply();//return a tree
```

### Algorithms (Part 2):
```cpp
//Given the graphs
UndirectedGraph<char, int> ugraph;
DirectedGraph<char, int> dgraph;

//1- Generates a tree using the BFS algorithm at any Vertex
BFS<char, int> bfsU(&ugraph);
UndirectedGraph<char, int> resultU = bfsU.apply(id);//return a tree
BFS<char, int> bfsD(&dgraph);
DirectedGraph<char, int> resultD = bfsD.apply(id);//return a tree

//2- Generates a tree using the DFS algorithm at any Vertex
DFS<char, int> dfsU(&ugraph);
UndirectedGraph<char, int> resultU = dfsU.apply(id);//return a tree
DFS<char, int> dfsD(&dgraph);
DirectedGraph<char, int> resultD = dfsD.apply(id);//return a tree

//3- Generates a table of distances and parents of the minimum path from one Vertex generated by the Dijkstra algorithm
Dijkstra<char, int> dijkstraU(&ugraph);
returnDijkstraType result = dijkstraU.apply(id); //returns a pair of unordered_map for distances and parents
Dijkstra<char, int> dijkstraD(&dgraph);
returnDijkstraType result = dijkstraD.apply(id); //returns a pair of unordered_map for distances and parents

//4- Generates a table of distances (G(n) and F(n)) and parents of the minimum path to one Vertex generated by the A* algorithm
AStar<char, int> astarU(&ugraph);
returnAStarType result = astarU.apply(idFrom, idTo, heuristic); //returns a pair of unordered_map for distances (G(n) and F(n)) and parents
AStar<char, int> astarD(&dgraph);
returnAStarType result = astarD.apply(idFrom, idTo, heuristic); //returns a pair of unordered_map for distances (G(n) and F(n)) and parents

//5- Generates a table of distances and parents of the minimum path from one Vertex generated by the Bellman-Ford algorithm
BellmanFord<char, int> bellmanfordU(&ugraph);
returnBellmanFordType result = bellmanfordU.apply(id); //returns a pair of unordered_map for distances and parents
BellmanFord<char, int> bellmanfordD(&dgraph);
returnBellmanFordType result = bellmanfordD.apply(id); //returns a pair of unordered_map for distances and parents

//6- Generates a matrix of distances and parents of the minimum path generated by the Floyd-Warshall algorithm
FloydWarshall<char, int> floydwarshallU(&ugraph);
returnFloydWarshallType result = floydwarshallU.apply(); //returns a matrix of unordered_map for distances and parents
FloydWarshall<char, int> floydwarshallD(&dgraph);
returnFloydWarshallType result = floydwarshallD.apply(); //returns a matrix of unordered_map for distances and parents

//7- Generates a list of strongly connected components (directed graphs) generated by the SCC algorithm. (only for directed graphs) 
SCC<char, int> scc(&dgraph);
returnSCCType result = SCC.apply(); //returns a list of strongly connected components (directed graphs)
```

## JSON file parser
* Construye un grafo a partir de una archivo JSON de aereopuertos del mundo. 


### Methods:
```cpp
void clear(); // Clears parser saved atributes

void readJSON(); // Parses JSON file and saves data into class
// NOTE: each derived class has its own readJSON method

void uGraphMake(UndirectedGraph<string, double> &tempGraph); // Adds the parsed data into the specified undirected graph

void dGraphMake(DirectedGraph<string, double> &tempGraph); // Adds the parsed data into the specified directed graph
```

## Graph Creator

* Se debe seleccionar si se crea un grafo dirigido o no dirigido.
* Debe permitir mostrar las características implementadas.
* Se deben aplicar los algoritmos creados.

## [Git Karma Guidelines](http://karma-runner.github.io/5.2/dev/git-commit-msg.html)

```
<type>(<scope>): <subject>

<body>
```

### Allowed ```<type>``` values

* feat (new feature for the user, not a new feature for build script)
* fix (bug fix for the user, not a fix to a build script)
* docs (changes to the documentation)
* style (formatting, missing semi colons, etc)
* refactor (refactoring production code, eg. renaming a variable)
* test (adding missing tests, refactoring tests)
* chore (updating grunt tasks etc)

### Allowed ```<scope>``` values

* graph
* directedGraph
* undirectedGraph
* parser
* main
* tester


> **PD:** Puntos extras sobre Evaluación Continua si se implementa una GUI.
