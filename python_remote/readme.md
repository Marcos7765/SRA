# Setup para uso da API remota do CoppeliaSim em Python via ZeroMQ

- O requisito mínimo para o cliente é que ele seja capaz de se comunicar via ZeroMQ, então você consegue fazer um cliente à mão se quiser. Aqui vamos usar o já feito em Python (na versão síncrona), mas já existem também clientes em C++, Java, Lua, Matlab/Octavte (direto do plugin), e Rust (third-party feito por um estudante da UFRN).

## Requisitos:

#### Cliente (python): (datado de 23/04/2025)
    - coppeliasim-zmqremoteapi-client; (testado na ver. 2.0.4)
    - Python >= 3.8; (testado na ver. 3.13.3)

#### Servidor:
    - CoppeliaSim (testado na versão Edu);
    - simZMQ; (Plugin do ZeroMQ para Coppelia, já vem instalado por padrão.)

OBS.: configurações gerais do Coppelia para Python podem precisar que você defina o `defaultPython` no arquivo `usrset.txt` na raíz da pasta do Coppelia e que instale algumas dependências. Assegure-se de ler todas as mensagens do console durante a inicialização (lá no início foi onde apareceram para mim nesses dois problemas).
## Configuração inicial:

### Servidor:

Note que o servidor não é um servidor para a simulação, mas sim um servidor para o Coppelia como um todo. Para o servidor:

1. Inicie o Coppelia
1. Carregue a cena que desejar
1. Verifique se o servidor remoto em ZeroMQ inicializou automaticamente:
    - Procure no console (abaixo da tela cena) por uma mensagem como `[Connectivity >> ZMQ remote API server@addOnScript:info] ZeroMQ Remote API server starting (rpcPort=23000)..."`;
    - Se não estiver inicializado, selecione `Modules->Connectivity->ZMQ remote API server` no menu do canto superior esquerdo;
    - Caso não esta opção não esteja disponível, provavelmente é sinal de que o simZMQ não está instalado.
    - (opcional) configurações do 
1. Garanta comunicação do Coppelia pelo firewall (dependente do sistema e da rede em que o cliente está)
1. (Opcional) carregue a cena que deseja ser simulada
    - Você pode navegar pelo simulador usando somente a API remota se quiser.


Para o cliente, assumindo que você já tenha a base do código, os passos se resumem a obter o ip e porta do servidor e usá-los na instanciação de RemoteAPIClient.

A API remota e a normal são muitíssimo semelhantes aos scripts globais (nível de sandbox ou plugin). Caso troque cenas, entretanto, o servidor ZMQ é reiniciado. A requisição de módulos do Coppelia ainda é por require e não tem IntelliSense. O PWD/CWD é o diretório raíz do coppelia ("<path normal de arquivos>/CoppeliaRobotics/CoppeliaSimEdu" no Windows ou na pasta extraída do .tar.xz no Linux).

Como a API é remota, é inevitável o delay entre simulação e programa. Para evitar isso, considere fazer a simulação por passos ao invés de tempo real (sim.set)

O `exemplo_zeromq.py` faz conexão, setup da cena e simulação de um robô de duas rodas diferencial caminhando em 8.