#include <matplot/matplot.h>

int main() {
    using namespace matplot;

    auto [x, y, z] = peaks();
    surf(x, y, z);
    colorbar().reverse(true);

    show();
    return 0;
}