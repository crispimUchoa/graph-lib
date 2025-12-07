from aux_func import ler_gr
from Digrafo import Digrafo
#Inicializa grafo
V, E = ler_gr('USA-road-d.NY.gr')

grafo = Digrafo(V, True)

for u, v, w in E:
    grafo.adicionar_aresta(u, v, w)
