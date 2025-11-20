class Grafo:
    def __init__(self, V: int, ponderado=False):
        self._V = V
        self._E = 0
        self._ponderado = ponderado
        self._matriz_adjacencia = [[0] * V for _ in range(V)]
        

    def mostrar_matriz(self):
        print(*self._matriz_adjacencia, sep='\n')

    def adicionar_aresta(self, u, v, w=1):
        peso = w
        if not self._ponderado:
            peso = 1   

        self._matriz_adjacencia[u][v] = peso
        self._matriz_adjacencia[v][u] = peso

        self._E+=1
    
    def n(self):
        return self._V
    
    def m(self):
        return self._E

    def viz(self, v):
        return [vizinho for vizinho in range(self._V) if self._matriz_adjacencia[v][vizinho] != 0]
    
    def d(self, v):
        return len(self.viz(v))

    def w(self, u:int, v:int):
        return self._matriz_adjacencia[u][v]
    
    def mind(self):
        minimo = 0
        for v in range(self._V):
            d = self.d(v)
            if d < minimo:
                minimo = d
        
        return minimo

    def maxd(self):
        maximo = 0
        for v in range(self._V):
            d = d(v)
            if d > maximo:
                maximo = d
        
        return maximo
    
    def bfs(self, s):
        d = []
        pi = []
        cor = []

        for _ in range(self._V):
            d.append(float('inf'))
            pi.append(None)
            cor.append('Branco')

        d[s] = 0
        cor[s] = 'Cinza'

        Q = []
        Q.append(s)
        
        while len(Q) > 0:
            u = Q.pop(0)
            for v in self.viz(u):
                if cor[v] == 'Branco':
                    cor[v] = 'Cinza'
                    d[v] = d[u] + self._matriz_adjacencia[u][v]
                    pi[v] = u
                    Q.append(v)
            cor[u] = 'Preto'
        
        return (d, pi)
    

    def dfs(self, s):
        tempo = 0
        ini = []
        fim = []
        pi = []
        cor = []

        def __busca_dfs(v):
            nonlocal tempo
            tempo +=1
            ini[v] = tempo
            cor[v] = 'Cinza'
            for u in self.viz(v):
                if cor[u] == 'Branco':
                    pi[u] = v
                    __busca_dfs(u)
            tempo+=1
            fim[v] = tempo
            cor[v] = 'Preto'

        for _ in range(self._V):
            ini.append(float('inf'))
            fim.append(float('inf'))
            pi.append(None)
            cor.append('Branco')

        __busca_dfs(s)

        return (pi, ini, fim)
        
    def bf(self, s):
        d = []
        pi = []

        for _ in range(self._V):
            d.append(float('inf'))
            pi.append(None)

        d[s] = 0

        for _ in range(self._V - 1):
            for u in range(self._V):
                for v in range( self._V):
                    if self._matriz_adjacencia[u][v] != 0:
                        if d[v] > d[u] + self._matriz_adjacencia[u][v]:
                            d[v] =d[u] + self._matriz_adjacencia[u][v]
                            pi[v] = u
        
        return (d, pi)
        
grafo = Grafo(6, ponderado=True)
grafo.adicionar_aresta(0, 1, 2)
grafo.adicionar_aresta(0, 2, 3)
grafo.adicionar_aresta(0, 3, 4)
grafo.adicionar_aresta(3, 4, -3)
grafo.adicionar_aresta(3, 5, 1)
grafo.adicionar_aresta(5, 3, 1)

grafo.mostrar_matriz()

print(grafo.bf(0))