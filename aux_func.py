def ler_gr(filename):
    num_vertices = 0
    arestas = []

    with open(filename, 'r') as f:
        for linha in f:
            linha = linha.strip()

            # Ignorar comentários e linhas vazias
            if not linha or linha.startswith('c'):
                continue

            partes = linha.split()

            # Linha com o número de vértices/arestas
            if partes[0] == 'p':
                # exemplo: p edge 10 20
                num_vertices = int(partes[2])

            # Aresta direcionada ponderada
            elif partes[0] == 'a':
                # Formato: a u v w
                _, u, v, w = partes
                u = int(u) - 1      # ajusta para começar em 0
                v = int(v) - 1
                w = int(w)       

                arestas.append((u, v, w))

    return num_vertices, arestas
