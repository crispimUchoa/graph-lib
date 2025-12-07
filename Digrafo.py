from Grafo import Grafo

class Digrafo(Grafo):
    def __init__(self, V: int, ponderado: bool = False):
        super().__init__(V, ponderado)

    def adicionar_aresta(self, u: int, v: int, w: int = 1): #aresta de u -> v
        peso = w
        if not self._ponderado:
            peso = 1   

        self._matriz_adjacencia[u][v] = peso

        self.__E +=1

    def viz(self, v: int) -> list[int]:
        vizinhos = []

        for u in range(self.n()):
            if self.w(v, u) != 0 or self.w(u, v) != 0 :
                vizinhos.append(u)
        
        return vizinhos



if __name__ == '__main__':
    digrafo = Digrafo(5)

    digrafo.mostrar_matriz()

