#pragma once

//template<typename V, typename H>
//apply
/*
template<typename V, typename H, typename... L, typename T>
typename std::result_of<typename T::operator()(typename V)>::type
apply(V v H h, L... l, T t) {
  return apply<V, L..., T>(h(std::move(v)));
}
*/
/*
template<typename V, typename H, typename... T>
struct result {
  typedef typename std::result_of<typename H::foo(V)>::type type;
};
*/
/*
template<typename H, typename... P>
struct Compose {
  explicit Compose(H h, P... p) : p_(h, p...)
  {}
  
  template<typename V>
  void operator()(V v) {
    
  }
  
  std::tuple<H, P...> p_;
};

namespace {
  struct A { std::string operator()(int ) { return "42"; } };
  struct B { double operator()(std::string ) { return -1; } };

  static auto y = Compose<A>(A());
  static auto z = Compose<A, B>(A(), B());

  static auto r = [] {
    std::cout << y(22) << std::endl;
    std::cout << z(44) << std::endl;
    return 44;
  }();
}

*/
