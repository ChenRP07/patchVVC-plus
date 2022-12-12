#include <iostream>
using namespace std;

void test_1() { 
    throw int(6);
}

void test() {
    try {
        test_1();
    }
    catch (int a) {
        cout << "test : " << a + 1 << endl;
        throw a+1;
    }
}

int main() {
    try {
        test();
    }
    catch (int a) {
        cout << "main : " << a << endl;
    }
    
}
