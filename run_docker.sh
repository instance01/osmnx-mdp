docker run -it yeeboi bash -c 'cd /app/osmnx_mdp && LD_PRELOAD="/app/lib.cpython-37m-x86_64-linux-gnu.so /app/cpp_mdp.cpython-37m-x86_64-linux-gnu.so"  python3 -c "import runner"'
