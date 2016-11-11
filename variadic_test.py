#!/usr/env/bin python3

def print_parameters(A, *arguments, key = None, **keyword_arguments):
    print('args =', arguments)
    print('kwargs =', keyword_arguments)

print_parameters(1,*[2,3],key=4, a=4,b=3,c='hoge')
print_parameters(1,[2,3],key=4, a=4,b=3,c='hoge')
