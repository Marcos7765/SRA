from typing import *
import inspect

class ControladorPID:
    def __init__(self, entrada:float, referencia:float, k_p:float, k_d:float, k_i:float, dt:float) -> None: ...
    def set_parametros(self, k_p:Optional[float] = None, k_i:Optional[float] = None, k_d:Optional[float] = None,
        dt:Optional[float] = None) -> None: ...
    def recalc_termos(self) -> None: ...
    def calc_saida(self) -> float: ...


    def __init__(self, entrada:float, referencia:float, k_p:float, k_d:float, k_i:float, dt:float) -> None:
        
        self.entrada = entrada
        self.referencia = referencia
        self.__k_p:float = k_p
        self.__k_d:float = k_d
        self.__k_i:float = k_i
        self.__dt:float = dt
        self.erros:List[float] = [0.,0.,0.]
        self.saidas:List[float] = [0., 0.]
        self.termos:List[float] = [0.,0.,0.]
        self.recalc_termos()

    #não precisei usar isso, então não cheguei a testar o funcionamento
    def set_parametros(self, k_p:Optional[float] = None, k_i:Optional[float] = None, k_d:Optional[float] = None,
        dt:Optional[float] = None):
        #aqui foi tempinho de pesquisa pra algo que era umas 4 linhas + 4 ctrl+c ctrl+v
        for nome, _ in inspect.signature(self.set_parametros).parameters.items():
            if locals()[nome] is None:
                continue
            setattr(self, f"_{self.__class__.__name__}__{nome}", locals()[nome])
        
        self.recalc_termos()

    def recalc_termos(self) -> None:
        self.termos[2] = self.__k_d/self.__dt
        self.termos[0] = self.__k_p + self.__k_i*self.__dt + self.termos[2]
        self.termos[1] = -self.__k_p - 2*self.termos[2]

    def calc_saida(self) -> float:
        self.erros[1:] = self.erros[:-1]
        self.saidas[1:] = self.saidas[:-1]
        self.erros[0] = self.referencia - self.entrada
        self.saidas[0] = self.saidas[1] + self.termos[0]*self.erros[0] + self.termos[1]*self.erros[1] + self.termos[2]*self.erros[2] #se eu tivesse usado o numpy nn teria essa bagunça

        return self.saidas[0]