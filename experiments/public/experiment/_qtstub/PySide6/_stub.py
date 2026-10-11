class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, name): return _Any()
    def __call__(self, *a, **k): return _Any()

class _Meta(type):
    def __getattr__(cls, name): return _Any()

class _Base(metaclass=_Meta):
    def __init__(self, *a, **k): pass
    def __getattr__(self, name): return _Any()

def Signal(*a, **k): return _Any()

def __getattr__(name):          # any other symbol resolves to a stub class
    return type(name, (_Base,), {})
