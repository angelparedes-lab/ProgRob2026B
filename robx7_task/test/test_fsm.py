"""Pruebas del motor de maquina de estados. No necesitan ROS."""
from robx7_task.fsm import StateMachine


def test_runs_until_final_and_records_history():
    log = []
    m = StateMachine('A', 'C', on_transition=lambda a, b: log.append((a, b)))
    m.add_state('A', lambda: 'B')
    m.add_state('B', lambda: 'C')
    m.add_state('C', lambda: 'C')
    assert m.run() == 'C'
    assert m.history == ['A', 'B', 'C']
    assert log == [('A', 'B'), ('B', 'C')]


def test_loop_with_counter():
    count = {'n': 0}

    def work():
        count['n'] += 1
        return 'WORK' if count['n'] < 3 else 'END'

    m = StateMachine('WORK', 'END')
    m.add_state('WORK', work)
    m.add_state('END', lambda: 'END')
    m.run()
    assert count['n'] == 3


def test_stops_when_asked():
    m = StateMachine('A', 'Z')
    m.add_state('A', lambda: 'A')
    assert m.run(should_continue=lambda: False) == 'A'
