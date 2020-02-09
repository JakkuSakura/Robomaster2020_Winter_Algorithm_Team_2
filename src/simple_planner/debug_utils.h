#if !defined(DEBUG_UTILS)
#define DEBUG_UTILS




#define show_line_num() std::cerr << "thread " << std::this_thread::get_id() << " at " << __LINE__ << std::endl
#define CHECK(x)                   \
  try                              \
  {                                \
    (x).check();                   \
  }                                \
  catch (const std::exception &e)  \
  {                                \
    std::cerr << e.what() << '\n'; \
    show_line_num();               \
    abort();                       \
  }
#define show_vector(x) __show_vector(x, __LINE__, #x)
template <typename T>
inline void __show_vector(T x, int line_number, const char *s)
{
  std::cerr << s << " at line " << line_number << " has " << x.size() << " children: ";
  for (auto &&i : x)
  {
    std::cerr << i << ", ";
  }
  std::cerr << std::endl;
}

#endif // DEBUG_UTILS