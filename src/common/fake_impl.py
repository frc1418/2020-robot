from typing import get_type_hints


class FakeImpl:
    def __init__(self, *args, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __getattribute__(self, attr):
        return FakeImpl()

    def __call__(self, *args, **kwargs):
        return FakeImpl(*args, **kwargs)
