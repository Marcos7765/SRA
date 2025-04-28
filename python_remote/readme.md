# Setup para uso da API remota do CoppeliaSim em Python via ZeroMQ

- O requisito mínimo para o cliente é que ele seja capaz de se comunicar via ZeroMQ, então você consegue fazer um cliente à mão se quiser. Aqui vamos usar o já feito em Python (na versão síncrona), mas já existem também clientes em C++, Java, Lua, Matlab/Octavte (direto do plugin), e Rust (third-party feito por um estudante da UFRN).

## Requisitos:

#### Cliente (python): (datado de 23/04/2025)
    - coppeliasim-zmqremoteapi-client; (testado na ver. 2.0.4)
    - Python >= 3.8; (testado na ver. 3.13.3)

#### Servidor:
    - CoppeliaSim (testado na versão Edu);
    - simZMQ; (Plugin do ZeroMQ para Coppelia, já vem instalado por padrão.)

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