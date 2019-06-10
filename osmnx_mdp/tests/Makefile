default:
	g++ -I../algorithms -I. cpp_test_main.cpp -c

tests:
	g++ -I../algorithms -I. cpp_test_main.o ../serialize_util.cpp ../algorithms/cpp_mdp.cpp ../algorithms/cpp_brtdp.cpp ../cpp_lib.cpp cpp_test_mdp.cpp cpp_test_brtdp.cpp -o cpp_test
	./cpp_test
