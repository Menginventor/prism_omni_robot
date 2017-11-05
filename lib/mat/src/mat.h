#include <mbed.h>
extern Serial*  mat_debug_port;

//Serial*  mat_debug_port = NULL;
class mat{
public:
  unsigned int mat_id = 0;
  int row, col;
  float** mat_data = 0;
  mat(int _row,int _col);
  ~mat();

  void print();
  mat operator+(const mat&  other);
  mat operator+(const float&  other);
  mat operator*(const mat&  other);
  mat operator*(const float&  other);
  void all_fill(float n);
  void array_copy(float *A);
  float det();
  float minor(int _row,int _col);
  float cofactor(int _row,int _col);

  mat transpose();
  mat adjoint();
  mat inverse();

  void set_to_I();
  void set_to_Rz(float theta);
};
