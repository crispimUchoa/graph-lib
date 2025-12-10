from MinHeap import MinHeap
import matplotlib.pyplot as plt

class Grafo:
    def __init__(self, V: int, ponderado:bool=False):
        self._V: int = V #número de vertices 
        self._E: int = 0 #número de arestas
        self._ponderado: bool = ponderado
        # self._matriz_adjacencia:list[list[int]] = [[0] * V for _ in range(V)] #cria matriz de adjacências para 0 arestas
        self._list_adjacencia: list[list[tuple[int, int]]] = [[] for _ in range(V)]
        

    # def mostrar_matriz(self):
    #     print(*self._matriz_adjacencia, sep='\n')

    def mostrar_lista(self):
        for i in range(self._V):
            print(i, '->', self._list_adjacencia[i])

    def adicionar_aresta(self, u:int, v:int, w:int=1):#adiciona aresta e atualiza matriz de adjacências
        peso = w
        if not self._ponderado:
            peso = 1   

        # self._matriz_adjacencia[u][v] = peso
        # self._matriz_adjacencia[v][u] = peso
        self._list_adjacencia[u].append((v, peso))
        self._list_adjacencia[v].append((u, peso))
        self._E+=1 #atualiza numero de arestas
    
    def n(self) -> int: #retorna numero de vertices
        return self._V
    
    def m(self) -> int: #retorna numero de arestas
        return self._E

    def w(self, u:int, v:int) -> int: #retorna peso de uv
        # return self._matriz_adjacencia[u][v]
        for vi, wi in self._list_adjacencia[u]:
            if vi == v:
                return wi
        return 0
    
    def viz(self, v: int) -> list[int]: #retorna vizinhos de v
        # return [vizinho for vizinho in range(self.n()) if (self.w(v, vizinho) != 0 and v != vizinho)]
        return [u for u, _ in self._list_adjacencia[v]]
    
    def d(self, v: int) -> int: #retorna grau de v
        return len(self.viz(v))

    
    def mind(self) -> int: #retorna grau mínimo de G
        minimo = self.d(0)
        for v in range(self.n()): #percorre todos os vertices calculando seus graus
            d = self.d(v)
            if d < minimo:
                minimo = d
        
        return minimo

    def maxd(self) -> int: #retorna grau máximo de G
        maximo = self.d(0)
        for v in range(self.n()):
            d = self.d(v)
            if d > maximo:
                maximo = d
        
        return maximo
    
    
    def inicializa(self)  -> tuple[list[float], list[int | None]]:#Inicializa G (d e pi)
        d: list[float] = []
        pi: list[int | None] = []
        
        for _ in range(self.n()): 
            d.append(float('inf'))
            pi.append(None)
        
        return (d, pi)

    def inicializa_busca(self) -> tuple[list[float], list[int | None], list[str]]:#inicializa algoritmos BFS e DFS
        d, pi = self.inicializa()
        cor: list[str] = ['Branco'] * self.n()
        return (d, pi, cor)
    
    def relaxa(self, u: int, v: int, d: list[float], pi: list[int | None]) -> bool: #Relaxa a aresta -> retorna verdadeiro se relaxou
        if self.w(u, v) != 0:
            if d[v] > d[u] + self.w(u, v):
                d[v] =d[u] + self.w(u, v)
                pi[v] = u
                return True
            
        return False


    def bfs(self, s: int) -> tuple[list[float], list[int|None]]: #Busca em largura
        d, pi, cor = self.inicializa_busca()

        d[s] = 0
        cor[s] = 'Cinza'

        Q = []  #Cria fila vazia e add s
        Q.append(s)
        
        while len(Q) > 0: #Enquanto fila não for vázia
            u = Q.pop(0)  #Desinfileira u
            for v in self.viz(u): #percorre todos os vizinhos de u
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
        for u in self.viz(v):
            if cor[u] == 'Branco':
                pi[u] = v
                tempo = self._busca_dfs(u, tempo, ini, fim, cor, pi)
        tempo+=1
        fim[v] = tempo
        cor[v] = 'Preto'
        return tempo

    def dfs_recursivo(self, s:int) -> tuple[list[int | None], list[int], list[int]]: #busca em profundidade
        ini:list[int] = [-1]*self.n()
        fim:list[int] = [-1]*self.n()
        pi, cor = self.inicializa_busca()[1:]
        self._busca_dfs(s, 0, ini, fim, cor,  pi) #Realiza a busca a partir de s

        return (pi, ini, fim)

    def _reconstruir_ciclo(self, v, u, pi):
        ciclo = [u]
        x = v

        visitados = set([u])
        while x not in visitados:
            visitados.add(x)
            ciclo.append(x)
            x = pi[x]
            if x is None:
                return []

        ciclo.append(x)
        ciclo.reverse()
        return ciclo


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

            viz = self.viz(v)

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

    def bf(self, s: int) -> tuple[list[float], list[int | None]]: #Bellman-Ford
        d, pi = self.inicializa()
        
        d[s] = 0

        for _ in range(self.n() - 1):
            for u in range(self.n()):
                for v in range( self.n()):
                    self.relaxa(u, v, d, pi)
        
        return (d, pi)
    
    def dijkstra(self, s:int) -> tuple[list[float], list[int | None]]: #algoritmo de dijkstra
        d, pi = self.inicializa()
        d[s] = 0
        heap = MinHeap(self.n(), d)
        heap.constroi_heap()
        while heap.heap:
            # print(heap.heap)
            u = heap.extrai_min()   
            # print(heap.heap)
            for v in self.viz(u):
                if self.relaxa(u, v, d, pi):
                    heap.reduz_distancia(v, d[v])


        return (d, pi)
        
    def coloracao_propria(self): #Algoritmo de wesh-powell + verificação de grafo completo
        n = self.n()
        
        V = list(range(n))
        V.sort(key=lambda v : -self.d(v)) #ordena os vertices por ordem decrescente de grau
        cor = 1 #inicia a cor 1
        cores = [0] * n #define cores, cores[i] cor do vertice V[i]
        adjancentes =[ set(self.viz(v)) for v in range(n)]
        nao_coloridos = set(V)
        while nao_coloridos: #Enquanto houver vértice incolor
            v = V.pop(0)
            cores[v] = cor

            adjancentes_marcados = adjancentes[v].copy()

            for u in list(nao_coloridos):
                if not u in adjancentes_marcados:
                    cores[u] = cor
                    nao_coloridos.remove(u)
                    adjancentes_marcados |= adjancentes[u]
            
            cor +=1

        return (cores, cor-1)
        

    # def desenhar_grafo(self): #exibe o grafo com matplotlib
    #     n = len(self._matriz_adjacencia)

    #     R = 1.0
    #     coords = [
    #         (R*math.cos(2*math.pi*i/n), R*math.sin(2*math.pi*i/n))
    #         for i in range(n)
    #     ]

    #     for i in range(n):
    #         for j in range(i+1, n):
    #             if self.w(i, j) != 0:
    #                 x1, y1 = coords[i]
    #                 x2, y2 = coords[j]
    #                 plt.plot([x1, x2], [y1, y2], linewidth=1, color="black")

    #     xs = [coord[0] for coord in coords]
    #     ys = [coord[1] for coord in coords]

    #     plt.scatter(xs, ys, s=600, color="white", edgecolors="black")

    #     for i, (x, y) in enumerate(coords):
    #         plt.text(x, y, str(i), ha='center', va='center', fontsize=12)

    #     plt.axis('equal')
    #     plt.axis('off')
    #     plt.show()



if __name__ == '__main__':
    grafo = Grafo(11, True)
    grafo.adicionar_aresta(0, 1, 15)
    grafo.adicionar_aresta(0, 7, 1)
    grafo.adicionar_aresta(1, 2, 3)
    grafo.adicionar_aresta(2, 3, 4)
    grafo.adicionar_aresta(4, 3, 7)
    grafo.adicionar_aresta(4, 1, 1)
    grafo.adicionar_aresta(5, 4, 1)
    grafo.adicionar_aresta(6, 1, 6)
    grafo.adicionar_aresta(6, 5, 1)
    grafo.adicionar_aresta(7, 6, 8)
    grafo.adicionar_aresta(7, 8, 5)
    grafo.adicionar_aresta(7, 10, 3)
    grafo.adicionar_aresta(8, 0, 4)
    grafo.adicionar_aresta(9, 6, 2)
    grafo.adicionar_aresta(10, 9, 1)

    grafo.mostrar_lista()

    v = 0
    print('\n----------- BFS ----------\n')

    d_bfs, pi_bfs = grafo.bfs(v)
    print(f'{d_bfs=}\n{pi_bfs=}')

    print('\n----------- DFS ----------\n')

    pi_dfs, ini_dfs, fim_dfs, ciclos = grafo.dfs(v)
    print(f'{pi_dfs=}\n{ini_dfs=}\n{fim_dfs=}')

    print('\n----------- Bellman-Ford ----------\n')

    d_bf, pi_bf = grafo.bf(v)
    print(f'{d_bf=}\n{pi_bf=}')

    print('\n----------- Djikstra ----------\n')

    d_dijkstra, pi_dijkstra = grafo.dijkstra(v)
    print(f'{d_dijkstra=}\n{pi_dijkstra=}')

    print('\n----------- Coloração ----------\n')

    cores, k = grafo.coloracao_propria()
    print(f'{cores=}\n{k=}')

    # grafo.desenhar_grafo()

    print(grafo.mind())
