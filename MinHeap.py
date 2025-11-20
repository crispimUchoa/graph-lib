class MinHeap: #Heap de mínimo de vértices com base em sua distância de s
    def __init__(self, V: int, d: list[float]): #V -> lista de vertices, d -> lista de distancias
        self.pos = list(range(V))
        self.d = d
        self.heap = list(range(V))
        self.constroi_heap()

    def swap(self, i, j):
        self.pos[self.heap[i]] = j
        self.pos[self.heap[j]] = i
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]

    def esq(self, i:int) -> int: #retorna o filho a esquerda
         return 2*i + 1
    
    def dir(self, i:int) -> int: #retorna o filho a direita
         return 2*i + 2
    
    def pai(self, i:int) -> int: #retorna o pai
         return (i - 1) //2
         
    def heapify_down(self, i:int):
        while True:
            esq = self.esq(i)
            dir = self.dir(i)
            menor = i #menor elemento entre d[i], d[esq(i)] e d[dir(i)]

            if esq < len(self.heap) and self.d[self.heap[esq]] < self.d[self.heap[menor]]:
                menor = esq

            if dir < len(self.heap) and self.d[self.heap[dir]] < self.d[self.heap[menor]]:
                menor = dir


            if menor != i:
                self.swap(i, menor)
                i = menor
            else:
                break

    def heapify_up(self, i:int):
        while i > 0:
            pai = self.pai(i)
            if self.d[self.heap[i]] < self.d[self.heap[pai]]:
                self.swap(i, pai)
                i = pai
            else:
                break

    def constroi_heap(self):
        for i in range(len(self.heap)//2 - 1, -1, -1 ):
            self.heapify_down(i)

    def reduz_distancia(self, u:int, d: float): #reajusta o heap quando há relaxamento de aresta
            i = self.pos[u]
            self.d[u] = d
            self.heapify_up(i)


    def extrai_min(self) -> int:
        u = self.heap[0]
        v = self.heap.pop()

        if len(self.heap) > 0:
            self.heap[0] = v
            self.pos[v] = 0
            self.heapify_down(0)

        self.pos[u] = -1

        return u