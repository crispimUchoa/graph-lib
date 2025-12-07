from Grafo import Grafo

class Digrafo(Grafo):
    def __init__(self, V: int, ponderado: bool = False):
        super().__init__(V, ponderado)


if __name__ == '__main__':
    digrafo = Digrafo(5)

    digrafo.mostrar_matriz()