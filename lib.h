#ifndef LIB_H
#define LIB_H

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <list>
#include <limits>
#include <math.h> 
#include <queue>
#include <set>
#include <stack>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#define distanceMatrixType std::map<std::string, std::map<std::string, TE>>
#define pathMatrixType std::map<std::string, std::map<std::string, std::string>>
#define returnFloydWarshallType std::pair<distanceMatrixType, pathMatrixType>

#define distanceUnorderedMapType std::unordered_map<Vertex<TV, TE>*, TE>
#define distanceUnorderedMapAStarType std::unordered_map<Vertex<TV, TE>*, std::pair<TE, TE>>
#define parentUnorderedMapType std::unordered_map<Vertex<TV, TE>*, Vertex<TV, TE>*>

#define returnDijkstraType std::pair<parentUnorderedMapType, distanceUnorderedMapType>
#define returnAStarType std::pair<distanceUnorderedMapAStarType, parentUnorderedMapType>
#define returnBellmanFordType std::pair<parentUnorderedMapType, distanceUnorderedMapType>
#define returnSCCType std::list<DirectedGraph<TV, TE>*>

bool check(int option, int inf, int sup){
    return option >= inf && option <= sup;
}

#ifdef _WIN32
  #include<windows.h>
#endif  

void console_clear(){
  #ifdef _WIN32
    system("cls");
  #else
    system("clear");
  #endif
}

void open_image(const char img_path[]){
    #ifdef _WIN32
        system(".\\" + img_path);
    #else
        system(&"xdg-open" [ *img_path]);
    #endif
}


void isConnectedMsg(bool cond){
    if(cond) std::cout << "\nIs Connected\n";
    else std::cout << "\nIs Disconnected\n";
}

int validInt(){
    int option;
    while(!(std::cin >> option)){
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input.  Try again: ";
    }
    return option;
}

void pause(){
    std::cout << "\nInserte cualquier caracter para continuar...\n";
    std::string c; std::cin>>c;
    console_clear();
}

void pause(std::string msg){
    std::cout << "\nInserte cualquier caracter para " << msg << " ...\n";
    std::string c; std::cin>>c;
    console_clear();
}

int menuMain(){
    int option;
    do{
        std::cout << "================================================\n";
        std::cout << "MENU GRAPH TESTER\n";
        std::cout << "================================================\n";    
        std::cout << "\t1. Test graph functions.\n";
        std::cout << "\t2. Test graph algorithms.\n";
        std::cout << "\t3. Test graph parser with pe.json.\n";
        std::cout << "\t4. Test graph parser with airports.json.\n";
        std::cout << "\t5. Create your graph\n";
        std::cout << "\t6. Exit\n";
        std::cout << "\nSelect option: ";
        option = validInt();
        console_clear();
    }while(!check(option, 1, 6));
    return option;
}

int menu1(){
    int option;
    do{
        std::cout << "================================================\n";
        std::cout << "MENU GRAPH FUNCTIONS TESTER\n";
        std::cout << "================================================\n";    
        std::cout << "\t1. Test density.\n";
        std::cout << "\t2. Test isConnected in Undirected Graph.\n";
        std::cout << "\t3. Test isStronglyConnected in Directed Graph.\n";
        std::cout << "\t4. Test isBipartite.\n";
        std::cout << "\t5. Test isWeaklyConnected in Directed Graph.\n";
        std::cout << "\t6. Back\n";
        std::cout << "\nSelect option: ";
        option = validInt();
        console_clear();
    }while(!check(option, 1, 6));
    return option;
}

void menu2(){
    std::cout << "================================================\n";
    std::cout << "MENU GRAPH ALGORITHMS TESTER\n";
    std::cout << "================================================\n";    
}

void menu3(){
    std::cout << "================================================\n";
    std::cout << "MENU GRAPH PARSER - PERU'S AIRPORTS\n";
    std::cout << "================================================\n";    
}

void menu4(){
    std::cout << "================================================\n";
    std::cout << "MENU GRAPH PARSER - WORLD'S AIRPORTS\n";
    std::cout << "================================================\n";    
}

void menu5(){
    std::cout << "================================================\n";
    std::cout << "MENU GRAPH CREATOR\n";
    std::cout << "================================================\n";    
}
#endif