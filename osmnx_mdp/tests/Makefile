default:
	g++ -I../algorithms -I. cpp_test_main.cpp -c

tests:
	g++ -I../external -I. cpp_test_main.o ../cpp_lib.cpp ../serialize_util.cpp ../algorithms/cpp_mdp.cpp cpp_test_mdp.cpp -o cpp_test_mdp.run
	g++ -I../external -I. cpp_test_main.o ../cpp_lib.cpp ../serialize_util.cpp ../algorithms/cpp_mdp.cpp ../algorithms/cpp_brtdp.cpp cpp_test_brtdp.cpp -o cpp_test_brtdp.run
	g++ -I../external -I. cpp_test_main.o ../cpp_lib.cpp ../serialize_util.cpp ../algorithms/cpp_dstar_lite.cpp cpp_test_dstar.cpp -o cpp_test_dstar.run
	./cpp_test_mdp.run
	./cpp_test_brtdp.run
	./cpp_test_dstar.run
