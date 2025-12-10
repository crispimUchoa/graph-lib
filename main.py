from aux_func import ler_gr
from Digrafo import Digrafo
from time import time
#Inicializa grafo
V, E = ler_gr('USA-road-d.NY.gr')

grafo = Digrafo(V, True)

for u, v, w in E:
    grafo.adicionar_aresta(u, v, w)
s = time()
print('Mind:', grafo.mind())
print('Maxd:', grafo.maxd())

print('Coloração própria com k =' ,grafo.coloracao_propria()[1])
d, pi = grafo.dijkstra(128)

maior_dist = 0
v_mais_dist = 128
for i in range(grafo.n()):
    if d[i] > maior_dist:
        maior_dist = d[i]
        v_mais_dist = i
print(f'O vértice mais distante de 129 é {v_mais_dist + 1}, com {maior_dist} de distância') 
inicio = 90643
caminho = [inicio,]
for _ in range(9):
    inicio = pi[inicio]
    if inicio == None:
        break
    caminho.append(inicio)

caminho.reverse()
print('Caminho com 10 vértices: ', *[f'-> {c+1}' for c in caminho])

#ACHA O CICLO COM TAMANHO MAIOR OU IGUAL A 5 ARESTAS
_,_,_,ciclos = grafo.dfs(0)
ciclo = []
for c in ciclos:
    if len(c) >=5:
        ciclo = c
        break
ciclo = [f'-> {c + 1}' for c in ciclo]

print('Ciclo com 5 ou mais arestas:',*ciclo)

e = time() - s
print(f'Tempo de execução: {round(e, 2)}s')