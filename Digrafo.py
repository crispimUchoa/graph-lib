from Grafo import Grafo
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

class Digrafo(Grafo):
    def __init__(self, V: int, ponderado: bool = False):
        super().__init__(V, ponderado)
        self._list_adjacencia_entrada = deepcopy(self._list_adjacencia)

    def adicionar_aresta(self, u: int, v: int, w: int = 1): #aresta de u -> v
        peso = w
        if not self._ponderado:
            peso = 1   

        self._list_adjacencia[u].append((v, peso))
        self._list_adjacencia_entrada[v].append((u, peso))

        self._E +=1

    def viz_saida(self, v:int) -> list[int]:
        return [u for u, _ in self._list_adjacencia[v]]
    
    def viz_entrada(self, v:int) -> list[int]:
        return [u for u, _ in self._list_adjacencia_entrada[v]]

    def viz(self, v: int) -> list[int]:
        return list(set(self.viz_entrada(v)) | set(self.viz_saida(v)))


    def d_saida(self, v:int) -> int:
        return len(self.viz_saida(v))
    
    def d_entrada(self, v:int) -> int:
        return len(self.viz_entrada(v))

    def d(self, v:int) -> int:
        return self.d_entrada(v) + self.d_saida(v)

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

    def dfs(self, s: int): #Iterativo

        ini = [-1] * self.n()
        fim = [-1] * self.n()
        pi, cor = self.inicializa_busca()[1:]
        ciclos = []

        pilha = [(s, 0)]  # (vértice, índice do vizinho)
        tempo = 0

        while pilha:
            v, idx = pilha.pop()

            if idx == 0:
                # Primeira vez que vemos v
                tempo += 1
                ini[v] = tempo
                cor[v] = 'Cinza'

            viz = self.viz_saida(v)

            if idx < len(viz):
                u = viz[idx]

                # Reempilha v para continuar depois do vizinho idx
                pilha.append((v, idx + 1))

                if cor[u] == 'Branco':
                    pi[u] = v
                    pilha.append((u, 0))

                elif cor[u] == 'Cinza':
                    # Tentou voltar para um vértice já visitado, um ciclo
                    ciclo = self._reconstruir_ciclo(v, u, pi)
                    if len(ciclo) == 10:
                        ciclos.append(ciclo)

            else:
                # Finalizando v
                cor[v] = 'Preto'
                tempo += 1
                fim[v] = tempo

        return pi, ini, fim, ciclos

    def desenhar_grafo(self):

        num_vertices = self.n()

        # Gerar posições circulares para os vértices
        angulos = np.linspace(0, 2*np.pi, num_vertices, endpoint=False)
        pos = {i: (np.cos(a), np.sin(a)) for i, a in enumerate(angulos)}

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect('equal')
        ax.axis('off')

        # Desenhar vértices
        for v, (x, y) in pos.items():
            ax.scatter(x, y, s=500, color='white', edgecolor='black', zorder=3)
            ax.text(x, y, str(v), ha='center', va='center', fontsize=12)

        # Desenhar arestas (percorrendo a matriz)
        for u in range(num_vertices):
            for v in range(num_vertices):
                w = self.w(u, v)
                if w != 0:  # existe aresta u -> v
                    x1, y1 = pos[u]
                    x2, y2 = pos[v]

                    dx = x2 - x1
                    dy = y2 - y1

                    # Seta
                    ax.arrow(
                        x1, y1,
                        dx * 0.75, dy * 0.75,
                        head_width=0.07,
                        length_includes_head=True,
                        fc='black', ec='black'
                    )

                    # Texto do peso
                    xm = (x1 + x2) / 2
                    ym = (y1 + y2) / 2
                    ax.text(xm, ym, str(w), fontsize=10, color='red')

        plt.show()

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

    digrafo.mostrar_lista()

    v = 0
    print('\n----------- BFS ----------\n')

    d_bfs, pi_bfs = digrafo.bfs(v)
    print(f'{d_bfs=}\n{pi_bfs=}')

    print('\n----------- DFS ----------\n')

    pi_dfs, ini_dfs, fim_dfs, ciclo = digrafo.dfs(v)
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

    # digrafo.desenhar_grafo()

    print(digrafo.mind())