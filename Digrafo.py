from Grafo import Grafo

class Digrafo(Grafo):
    def __init__(self, V: int, ponderado: bool = False):
        super().__init__(V, ponderado)

    def adicionar_aresta(self, u: int, v: int, w: int = 1): #aresta de u -> v
        peso = w
        if not self._ponderado:
            peso = 1   

        self._matriz_adjacencia[u][v] = peso

        self._E +=1

    def viz_saida(self, v:int) -> list[int]:
        return [vizinho for vizinho in range(self.n()) if (self.w(v, vizinho) != 0 and v != vizinho)]
    
    def viz_entrada(self, v:int) -> list[int]:
        return [vizinho for vizinho in range(self.n()) if (self.w(vizinho, v) != 0 and v != vizinho)]

    def viz(self, v: int) -> list[int]:
        vizinhos = self.viz_entrada(v) + self.viz_saida(v)
        
        return list(set(vizinhos))

    def d_saida(self, v:int) -> int:
        return len(self.viz_saida(v))
    
    def d_entrada(self, v:int) -> int:
        return len(self.viz_entrada(v))

    def bfs(self, s: int) -> tuple[list[float], list[int|None]]: #Busca em largura
        d, pi, cor = self.inicializa_busca()

        d[s] = 0
        cor[s] = 'Cinza'

        Q = []  #Cria fila vazia e add s
        Q.append(s)
        
        while len(Q) > 0: #Enquanto fila não for vázia
            u = Q.pop(0)  #Desinfileira u
            for v in self.viz_saida(u): #percorre todos os vizinhos de saída de u
                if cor[v] == 'Branco':
                    cor[v] = 'Cinza'
                    d[v] = d[u] + self.w(u, v)
                    pi[v] = u
                    Q.append(v)
            cor[u] = 'Preto'
        
        return (d, pi)

    def _busca_dfs(self, v:int, tempo:int, ini:list[int], fim:list[int], cor: list[str], pi:list[int | None]): #Faz a busca recursiva a partir de v
        tempo +=1
        ini[v] = tempo
        cor[v] = 'Cinza'
        for u in self.viz_saida(v):
            if cor[u] == 'Branco':
                pi[u] = v
                tempo = self._busca_dfs(u, tempo, ini, fim, cor, pi)
        tempo+=1
        fim[v] = tempo
        cor[v] = 'Preto'
        return tempo

if __name__ == '__main__':
    digrafo = Digrafo(11, True)
    digrafo.adicionar_aresta(0, 1, 15)
    digrafo.adicionar_aresta(0, 7, 1)
    digrafo.adicionar_aresta(1, 2, 3)
    digrafo.adicionar_aresta(2, 3, 4)
    digrafo.adicionar_aresta(4, 3, 7)
    digrafo.adicionar_aresta(4, 1, 1)
    digrafo.adicionar_aresta(5, 4, 1)
    digrafo.adicionar_aresta(6, 1, 6)
    digrafo.adicionar_aresta(6, 5, 1)
    digrafo.adicionar_aresta(6, 7, 4)
    digrafo.adicionar_aresta(7, 6, 8)
    digrafo.adicionar_aresta(7, 8, 5)
    digrafo.adicionar_aresta(7, 10, 3)
    digrafo.adicionar_aresta(8, 0, 4)
    digrafo.adicionar_aresta(9, 6, 2)
    digrafo.adicionar_aresta(10, 9, 1)

    digrafo.mostrar_matriz()

    v = 0
    print('\n----------- BFS ----------\n')

    d_bfs, pi_bfs = digrafo.bfs(v)
    print(f'{d_bfs=}\n{pi_bfs=}')

    print('\n----------- DFS ----------\n')

    pi_dfs, ini_dfs, fim_dfs = digrafo.dfs(v)
    print(f'{pi_dfs=}\n{ini_dfs=}\n{fim_dfs=}')

    print('\n----------- Bellman-Ford ----------\n')

    d_bf, pi_bf = digrafo.bf(v)
    print(f'{d_bf=}\n{pi_bf=}')

    print('\n----------- Djikstra ----------\n')

    d_dijkstra, pi_dijkstra = digrafo.dijkstra(v)
    print(f'{d_dijkstra=}\n{pi_dijkstra=}')

    print('\n----------- Coloração ----------\n')

    cores, k = digrafo.coloracao_propria()
    print(f'{cores=}\n{k=}')
