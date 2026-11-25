"""Motor minimo de maquina de estados finitos.

Archivo sin ROS. Lo usa task_manager.py, que registra aqui los estados del problema.

Una maquina de estados finitos tiene
  un conjunto finito de estados
  un estado inicial
  una funcion de transicion, que dado el estado actual y lo ocurrido decide el siguiente
  uno o mas estados finales

Aqui cada estado es una funcion sin argumentos que ejecuta su trabajo y devuelve el
nombre del estado siguiente. Asi la transicion queda escrita dentro del propio estado.
"""


class StateMachine:
    def __init__(self, initial, final, on_transition=None):
        self.initial = initial
        self.final = final
        self.states = {}
        self.current = initial
        self.on_transition = on_transition   # funcion opcional que avisa cada cambio de estado
        self.history = []                    # lista de estados visitados

    def add_state(self, name, handler):
        """Registra la funcion que ejecuta el estado name."""
        self.states[name] = handler

    def step(self):
        """Ejecuta el estado actual y avanza al siguiente. Devuelve el nombre nuevo."""
        if self.current not in self.states:
            raise KeyError(f'Estado sin definir {self.current}')
        previous = self.current
        self.history.append(previous)
        nxt = self.states[previous]()
        if nxt != previous and self.on_transition is not None:
            self.on_transition(previous, nxt)
        self.current = nxt
        return nxt

    def run(self, should_continue=lambda: True, max_steps=10000):
        """Ejecuta pasos hasta llegar al estado final o hasta que should_continue sea falso."""
        self.current = self.initial
        self.history = []
        steps = 0
        while self.current != self.final and should_continue() and steps < max_steps:
            self.step()
            steps += 1
        if self.current == self.final:
            self.history.append(self.final)
            if self.final in self.states:
                self.states[self.final]()
        return self.current
