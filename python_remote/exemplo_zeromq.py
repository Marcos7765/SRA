#as demais entradas do módulo são só pra suportar o RemoteAPIClient
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

RemoteAPIClient_parametros = dict(
    host='localhost', #IP do servidor, por padrão o da máquina local
    port=23000, #porta que o servidor está usando, por padrão 23000 (sim, em int)
    cntport=None, #não tem descrição e, no código fonte, não tem uso em momento algum.
        #Na versão assíncrona o padrão é port+1 (mas continua sem outra ocorrência no código)
    verbose=None #flag de verbosidade, por padrão lê da variável de ambiente VERBOSE (int(os.environ.get('VERBOSE', '0')))
)

cliente = RemoteAPIClient(RemoteAPIClient_parametros)

sim = cliente.require()