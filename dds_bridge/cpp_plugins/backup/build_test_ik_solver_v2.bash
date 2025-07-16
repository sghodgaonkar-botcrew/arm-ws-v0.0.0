export IPOPT_LIBRARY_PATH=/usr/lib && \
echo "Building test_ik_solver_v2..." && \
time g++ -std=c++17 -O2 -g \
    -include cstddef \
    $(pkg-config --cflags eigen3 pinocchio) \
    -I/usr/local/include/coin-or \
    -I/usr/include/x86_64-linux-gnu \
    -I. \
    test_ik_solver_v2.cpp \
    ik_solver_v2.cpp \
    ik_model.cpp \
    $(pkg-config --libs eigen3 pinocchio) \
    -L/usr/local/lib \
    -L/usr/lib/x86_64-linux-gnu/atlas \
    -lipopt \
    -llapack \
    -latlas \
    -lm \
    -lblas \
    -latlas \
    -lm \
    -o test_ik_solver_v2