auto t3 = high_resolution_clock::now();
  duration<double, std::milli> ms_double = t3 - t1;
  std::cout <<"Initialization is \t" << ms_double.count() << "ms\n";
