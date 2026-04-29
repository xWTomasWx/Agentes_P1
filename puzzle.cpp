#include "puzzleImpl.h"
#include "solverImpl.h"
#include <fstream>
#include <iostream>
#include <vector>

std::size_t NODELIMIT = 5000000;
int shuffleIterations = 25;

template<int N>
void runSolvers(int iters, std::ofstream& outFile){
    shuffleIterations = iters;
    P<N> p; 

    std::streambuf* originalCoutBuffer = std::cout.rdbuf();
    std::cout.rdbuf(outFile.rdbuf());

    greedy(p);
    BFS(p);
    AStar(p);
    IDAStar(p);
    BeamSearch(p, 100); 
    RBFS(p);
    SMAStar(p, 100000); 

    std::cout.rdbuf(originalCoutBuffer);
}

int main() {
    srand((unsigned)time(0));

    std::ofstream outFile("resultados_36.csv");
    if (!outFile.is_open()) {
        std::cerr << "Error al crear el archivo CSV.\n";
        return 1;
    }

    outFile << "Algoritmo,Tamano,Shuffle,NodosGenerados,NodosExpandidos,FronteraMaxima,FactorRamificacion,LongitudSolucion,Tiempo_ms\n";

    std::vector<int> shuffles = {25, 50, 100, 200};
    int repeticiones = 20;

    std::cout << "Iniciando simulaciones...\n";

    for (int rep = 1; rep <= repeticiones; ++rep) {
        for (int s : shuffles) {
            std::cout << "Procesando Repeticion " << rep << "/20 - Shuffle " << s << "...\n";
            
            // Variaciones
            //runSolvers<16>(s, outFile);
            //runSolvers<25>(s, outFile); 
            runSolvers<36>(s, outFile); 
        }
    }

    outFile.close();
    std::cout << "\n¡Todo listo! Revisa tu archivo 'resultados.csv'.\n";
    return 0;
}