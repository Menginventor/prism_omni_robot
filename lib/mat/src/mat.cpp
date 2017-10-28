#include <mat.h>

mat::mat(int _row,int _col){
        row = _row;
        col = _col;
        mat_data = new float*[row];

        for (int i = 0; i < row; i++) {
                mat_data [i] = new float [col];

                for (int j = 0; j < col; j++)
                {
                        mat_data [i][j] =0;
                }
        }
}

mat::~mat(){
      for (int i = 0; i < row; i++) {
              delete[] mat_data [i] ;

      }
      delete[] mat_data;
      //debug->printf("deleteed!!\n");
      //delete debug;
}

void mat::print(){
        debug->printf("row = %d , col = %d\n",row,col);
        for(int i = 0; i < row; i++) {      //row
                debug->printf("[");
                for(int j= 0; j < col; j++) {      //col
                        debug->printf("%f\t",mat_data[i][j] );
                }
                debug->printf("]\n");
        }
}
void mat::all_fill(float n){
        for(int i = 0; i < row; i++) { //row

                for(int j= 0; j < col; j++) { //col
                        mat_data[i][j] = n;
                }

        }
}
void mat::array_copy(float *A){
        for(int i = 0; i < row; i++) { //row

                for(int j= 0; j < col; j++) { //col
                        mat_data[i][j] = A[j+i*col];
                }

        }
}
mat mat::operator+(const mat&  other){
        mat result(row,col);
        for(int i = 0; i < row; i++) { //row
                for(int j= 0; j < col; j++) { //col
                        result.mat_data[i][j] = mat_data[i][j] +other.mat_data[i][j];
                }
        }
        return result;
}
mat mat::operator+(const float &  other){
        mat result(row,col);
        for(int i = 0; i < row; i++) { //row
                for(int j= 0; j < col; j++) { //col
                        result.mat_data[i][j] = mat_data[i][j] +other;
                }
        }
        return result;
}
mat mat::operator*(const mat &  other){

        mat result(row,other.col);

        if(col != other.row) {
                result.all_fill(-1);
                return result;
        }

        for(int i = 0; i < result.row; i++) { //row
                for(int j= 0; j < result.col; j++) { //col
                        result.mat_data[i][j] = 0;
                        for(int k= 0; k < col; k++) {
                                result.mat_data[i][j] += mat_data[i][k] * other.mat_data[k][j];
                                //result.mat_data[i][j]  = 5;
                        }
                }
        }
        return result;
}
mat mat::operator*(const float &  other){
        mat result(row,col);
        for(int i = 0; i < row; i++) { //row
                for(int j= 0; j < col; j++) { //col
                        result.mat_data[i][j] = mat_data[i][j] *other;
                }
        }
        return result;
}
float mat::det(){

        if(col == 1 && row ==1) {
                return mat_data[0][0];
        }
        else{
                //debug -> printf("more than 1 cell\n");
                float result = 0;
                for(int j = 0; j<col; j++) {
                        //debug -> printf("j = %d val = %f\n cof = %f",j,mat_data[0][j],cofactor(this,0,j));
                        //debug -> printf("cof [0][%d] = %f\n",j,cofactor(this,0,j));
                        /*if(debug  != NULL){
                           debug -> printf("aij = %f\n", mat_data[0][j]);
                           debug -> printf("cof = %f\n", cofactor(0,j));
                           }*/
                        result += mat_data[0][j]*cofactor(0,j);
                }
                return result;
        }

}
float mat::minor(int _row,int _col){
        mat M (row-1,col-1);
        for(int i = 0; i<row; i++) {
                if(i == _row) continue;

                for(int j = 0; j<col; j++) {
                        if(j == _col) continue;
                        if(i<_row) {
                                if(j<_col) {
                                        M.mat_data[i][j] = mat_data[i][j];
                                }
                                else{//j>col
                                        M.mat_data[i][j-1] = mat_data[i][j];
                                }
                        }
                        else{
                                if(j<_col) {
                                        M.mat_data[i-1][j] = mat_data[i][j];
                                }
                                else{//j>col
                                        M.mat_data[i-1][j-1] = mat_data[i][j];
                                }
                        }
                }
        }
        //debug->printf("mat M have row = %d col = %d\n",M->row,M->col);
        //M->print(debug);
        float result = M.det();

        return result;
}

float mat::cofactor(int _row,int _col){//  debug->printf("call cof\n");
        return pow(-1,_row+ _col)*minor(_row, _col);

}
mat mat::transpose(){
        mat result(row,col);
        result.debug = debug;
        for(int i = 0; i < row; i++) { //row
                for(int j= 0; j < col; j++) { //col
                        result.mat_data[j][i] = mat_data[i][j];
                }
        }
        return result;
}
mat mat::inverse(){

        return adjoint()*(1.0/det());
}
mat mat::adjoint(){
        mat result(row,col);
        result.debug = debug;
        for(int i = 0; i < row; i++) { //row
                for(int j= 0; j < col; j++) { //col
                        result.mat_data[j][i] = cofactor(i,j);
                }
        }
        return result;
}
void mat::set_to_I(){

  for(int i = 0; i < row; i++) {      //row

          for(int j= 0; j < col; j++) {      //col
                  if(i == j)mat_data[i][j] = 1.0;
                  else mat_data[i][j] = 0.0;
          }

  }
}
void mat::set_to_Rz(float theta){
  set_to_I();
  mat_data[0][0] = cos(theta);
  mat_data[0][1] = -sin(theta);
  mat_data[1][0] = sin(theta);
  mat_data[1][1] = cos(theta);
}
