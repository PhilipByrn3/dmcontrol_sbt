class Foo1():
    def __init__(self):
        pass
    def one(self):
        print('zoink')
    def two(self):
        Foo1.one()
        
f = Foo1()
f.one()