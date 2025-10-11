typedef struct point {
  double temp;
  double timestamp;
} Point;



double linearRegressionSlope(Point ps[], int n) {
  double sumx = 0.0;
  double sumy = 0.0;
  double sumxy = 0.0;
  double sumx2 = 0.0;
  int count = 0;

  for (int i = 0; i < n; ++i) {
    // 無効点のスキップ (x==0 && y==0)
    if (ps[i].timestamp == 0.0 && ps[i].temp == 0.0) continue;

    double xi = ps[i].timestamp;
    double yi = ps[i].temp;

    sumx += xi;
    sumy += yi;
    sumxy += xi * yi;
    sumx2 += xi * xi;
    ++count;
  }

  if (count < 2) {
    // データ不足
    return NAN;
  }

  double meanx = sumx / count;
  double meany = sumy / count;

  // 分子: Σ(xy) - N * meanx * meany
  double numerator = sumxy - (double)count * meanx * meany;
  // 分母: Σ(x^2) - N * meanx^2
  double denominator = sumx2 - (double)count * meanx * meanx;

  const double EPSp = 1e-12;
  if (fabs(denominator) < EPSp) {
    // x の分散が実質ゼロ -> 傾き決定不能
    return NAN;
  }

  return numerator / denominator;
}

void psclear(Point ps[], int n){
  for(int i = 0;i < n;i++){
    ps[i].timestamp = 0;
    ps[i].temp = 0;
  }
}