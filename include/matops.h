/*
 * matops.h
 *
 *  Created on: 2017. 6. 26.
 *      Author: Dongwoo Kim
 */

#ifndef BIN_MATOPS_H_
#define BIN_MATOPS_H_

float *vector(int,int);
float **matrix(int,int,int,int);
float **convert_matrix(float *,int,int,int,int);
double *dvector(int,int);
double **dmatrix(int,int,int,int);
int *ivector(int,int);
int **imatrix(int,int,int,int);
float **submatrix(float **,int,int,int,int,int,int);
void free_vector(float *,int,int);
void free_dvector(double *,int,int);
void free_ivector(int *,int,int);
void free_matrix(float **,int,int,int,int);
void free_dmatrix(double **,int,int,int,int);
void free_imatrix(int **,int,int,int,int);
void free_submatrix(float **,int,int,int,int);
void free_convert_matrix(float **,int,int,int,int);
void nrerror(char *);

void odeint(double *,int,double,double,double,double,double,int *,int *,
	    void (*derivs)(),void (*rkqc)());
void rkqc(double *,double *,int,double *,double,double,double *,double *,double *,
	  void (*derivs)());
void rk4(double *,double *,int,double,double,double *,void (*derivs)());
void bsstep(double *,double *,int,double *,double,double,double *,double *,
	    double *,void (*derivs)());
void mmid(double *,double,int,double,double,int,double *,void (*derivs)());
void rzextr(int,double,double *,double *,double *,int,int);

void four1(double *,int,int);
void realft(double *,int,int);
void cosft(double *,int,int);
void sinft(double *,int);
void twofft(double *,double *,double *,double *,int);
void correl(double *,double *,int,double *);
void convlv(double *,int,double *,int,int,double *);
void spctrm(double *,double *,int,int,int);
void memcof(double *,int,int,double *,double *);
double evlmem(double,double *,int,double);

void mrqmin(double *,double *,double *,int,double *,int,int *,int,double **,
	    double **,double *,void (*)(double,double *,double *,double *,int),
	    double *);
void mrqcof(double *,double *,double *,int,double *,int,int *,int,double **,
	    double *,double *,void (*)(double,double *,double *,double *,int));
void covsrt(double **,int,int *,int);
void gaussj(double **,int,double **,int);

//double ran0(int *);
//double ran1(int *);
//double ran2(long *);
//double ran3(int *);
//double ran4(int *);
//double gasdev(int *);
//double gamdev(int,int *);
//double bnldev(double,int,int *);
//double poidev(double,int *);
//double Gauss_dev(int *);
//
void white_phase(int,double *,double,int *);
void white_freq(int,double *,double,int *);
void rand_walk_freq(int,double *,double,int *);
//double A_var(int,double *,int);
//double M_var(int,double *,int);
//
//double gammln(double);
//double gammp(double,double);
//double gammq(double,double);
void gser(double *,double,double,double *);
void gcf(double *,double,double,double *);
//double erf(double);
//double erfc(double);
//double erfcc(double);
//double factrl(int);
//double bico(int,int);
//double factln(int);
//double beta(double,double);
//double Gammaf(double);
//double chi2_find(double,double);
//double x2_find(double,double);
//double GP_find(double,double);
//double Gauss_P(double);


double **dh2hth(double **H, int nrl, int nrh, int ncl, int nch);
double *matxvec(double **H,int nrl,int nrh,int ncl,int nch,double *b,int nel,int neh);
void dmatxmat(double **mat1, int nrl1, int ncl1, double **mat2, int nrl2, int ncl2, double **ans, int nr1, int nc1, int nc2);
void dprintmat(double **m, int  nrl, int nrh, int ncl, int nch);
double **dtranspose(double **m, int  nrl, int nrh, int ncl, int nch);
double *daddvec(double *v1, double coef1, int nel1, double *v2, double coef2, int nel2, int len);
double *scalxdvec(double scal, double *vec, int nel, int len);
double dvecnorm(double *vec, int nel,int len);
void dprintvec(double *v, int nel, int neh);
double ddot(double *vec1, int nel1, double *vec2, int nel2, int len);
double dtrace(double **A, int nrl, int ncl, int n);
double dpdop(double **H, int nr, int nc);
void daddvecv(double *v1, double coef1, int nel1, double *v2,
	      double coef2, int nel2,
	      double *v, int nelv, int len);

float cholfact(float **A,int n);
double dcholfact(double **A,int n);
void backsubL(float **L,float *x,float *b,int n);
void dbacksubL(double **L,double *x,double *b,int n);
void backsubLT(float **L,float *x,float *b,int n);
void cholbksb(float **L,float *x,float *b,int n);
void dcholbksb(double **L,double *x,double *b,int n);
int dbacksubLT(double **L,double *x,double *b,int n);
void dmatpinv(double **A, int nr, int nc, double **Api);
void dpinv(double **A, int nr, int nc, double **Api);
void enutdop(double **H, int nr, int nc, double *enutdopv);
void surv2enu(double dist, double azd, double azm, double azs,
	      double eld, double elm, double els,
	      double *eastp, double *northp, double *upp);
void dmake_rot_mat(double **R, double theta, int axis);
void dget_Att_Rot( double **A, double azi, double pit, double rol );
void dsyminv(double **ATA, int nc, double **Asi);

typedef struct {
  double **mat;
  int nrl; // # of row length
  int nrh;
  int ncl; // # of column length
  int nch;
} dmat;

#define ZERODMAT {(double **) 0, 0, 0, 0, 0}

/***BSP3***/

typedef struct {  /* Lower triangular matrix */
  double **mat;
  int size;
} Ldmat;

#define ZEROLDMAT {(double **)  0, 0, 0}

/**endBSP3***/


typedef struct {
  double *vec;
  int nel;
  int neh;
} dvec;

#define ZERODVEC {(double *) 0, 0, 0}

typedef struct {
  int *vec;
  int nel;
  int neh;
} ivec;

#define ZEROIVEC {(int *) 0, 0, 0}

//void initdmat(dmat *dmatp, int nrl, int nrh, int ncl, int nch);
//void freedmat(dmat *dmatp);
//void dmatxdmat(dmat *mat1p, dmat *mat2p, dmat *ansp);
void dmatxdmatsim(dmat *mat1p, dmat *mat2p, dmat *ansp);
//void dmatplsdmat(dmat *mat1p, dmat *mat2p, dmat *ansp);
int dmatinitialized(dmat *dmatp);
int dvecinitialized(dvec *dvecp);
int ivecinitialized(ivec *dvecp);
//void dmatxdvec(dmat *matp, dvec *vecp, dvec *ansp);
//void initdvec(dvec *dvecp, int nel, int neh);
//void freedvec(dvec *dvecp);
void initivec(ivec *ivecp, int nel, int neh);
void freeivec(ivec *ivecp);
//void setdmat(dmat *dmatp, int sub1, int sub2, double val);
//void setdvec(dvec *dvecp, int sub, double val);
void setivec(ivec *ivecp, int sub, int val);
//void printdmat(dmat *dmatp);
//void printdvec(dvec *dvecp);
void printivec(ivec *dvecp);
//void transdmat(dmat *matp, dmat *ansp);
void dmatsinv(dmat *ATAp, dmat *Asip);
//void dvecxscal(dvec *dvecp, double scal, dvec *ansp);
//void dmatxscal(dmat *dmatp, double scal, dmat *ansp);
void dmateye(dmat *dmatp, int n);
void subdmat(dmat *dmatp, int row1, int row2, int col1, int col2, dmat *ansp);
void subdvec(dvec *dvecp, int el1, int el2, dvec *ansp);
//void dvecplsdvec(dvec *dvec1p, dvec *dvec2p, dvec *ansp);
double dvecdotdvec(dvec *dvec1p, dvec *dvec2p);
double dvec2norm(dvec *vecp);
void dludcmp (double **a, int n, int *indx, double *d);
void dlubksb(double **a, int n, int *indx, double b[]);
//void zerodmat(dmat *dmatp);
void zerodvec(dvec *dvecp);
void reduce_obsv(dmat *Hp, dmat *Hrp, dmat *Tp, int n);
void dsim_trans(dmat *matp, dmat *transp, dmat *ansp);
void dcheck_mat_init(dmat *ansp, int nr, int nc, char *c);
int meas_consistent(dvec *zp, dmat *Hp, dmat *Rp, dvec *xp, dmat *Pp, float thresh);
void dvec_outer_product(dvec *vecp, dmat *ansp);
void Axebsolve(dmat *Ap, dvec *bp, dvec *xp);
void hth(dmat *Hp, dmat *HtHp);
void hht(dmat *Hp, dmat *HtHp);
void dmat2diag(dmat *matp, dvec *vecp);
void rotcov(dmat *cxp, dmat *Rp, dmat *cyp);
long int long_abs(long int i);
void make_rot_dmat(double theta, int axis, dmat *Rp);
//void dmatinv(dmat *ATAp, dmat *Asip);
void pinvdmat(dmat *Ap, dmat *Ainvp);
void hthinv(dmat *ap, dmat *ataip);
void dvecmindvec(dvec *vec1p, dvec *vec2p, dvec *ansp);
void blockmat(dmat *Ap, dmat *Bp, dmat *Cp, dmat *Dp, dmat *ansp);
int nrows(dmat *Ap);
int ncols(dmat *Ap);
//void setdmatrel(dmat *Ap, int relrow, int relcol, double value);
double getdmatrel(dmat *Ap, int relrow, int relcol);
double getdmat(dmat *Ap, int row, int col);
double getdvec(dvec *dvecp, int sub);
void copydmat(dmat *Ap, dmat *ansp);
void dopvec(dmat *Hp, dvec *vecp);
double pdop(dmat *Hp);
int getivec(ivec *ivecp, int sub);
void zeroivec(ivec *ivecp);

/*****BSP*******/

void copydvec(dvec *origp, dvec *copyp);
void copyivec(ivec *origp, ivec *copyp);
void Sxebsolve(dmat *Sp, dvec *bp, dvec *xp);
void SMeBsolve(dmat *Sp, dmat *Bp, dmat *Xp);
void subivec(ivec *ivecp, int el1, int el2, ivec *ansp);

/***endBSP*****/

void AtSA(dmat *Sp, dmat *Ap, dmat *ansp);

/***BSP3****/

void initLdmat(Ldmat *Ldmatp, int size);
void freeLdmat(Ldmat *Ldmatp);
void zeroLdmat(Ldmat *Ldmatp);
void Ldmat2dmat(Ldmat *Ldmatp, dmat *dmatp);
void dmat2Ldmat(dmat *dmatp, Ldmat *Ldmatp);

/***endBSP3***/

void AS2ASinv(dmat *Ap, dmat *Sp, dmat *ansp);
void Axebsolve2(dmat *Ap, dvec *bp, dvec *xp);

/***BSP9***/
double dmat_inf_norm(dmat *Ap);
/***endBSP9***/

void dsvdcmp(dmat *Ap, dvec *wp, dmat *Vp);
void svdcmp(double **a, int m,int n,double *w,double **v);
void null(dmat *Ap, dmat *Lp);
void left_null(dmat *Ap, dmat *Lp);
double weight_res(dvec *resp, dmat *Wip);

#endif /* BIN_MATOPS_H_ */
