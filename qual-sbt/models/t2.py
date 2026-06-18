class Person:
    VALID_KEYS = {
        'name',
        'age',
        'color',
        'temperature'
        }
    def __init__(self, **kwargs):
        invalid = set(kwargs) - self.VALID_KEYS
        if invalid:
            raise ValueError(f'Unknown Params: {invalid}')
        
        for key, value in kwargs.items():
            setattr(self, key, value)
            
        self.name = kwargs.get('name', 'FIRSTNAME LASTNAME')
        self.age = kwargs.get('age', 400)
            
s1 = Person(
    age='22'
)

print(s1.name)
