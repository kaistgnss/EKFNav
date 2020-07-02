#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include "matops.h"

void nrerror(char error_text[]) {
	void exit();
#ifdef __MSDOS__
	fcloseall();
#endif

	//  quitcurses();

	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);

	exit(1);
	/*  quit();*/
}

float *vector(int nl,int nh)
{
	float *v;

	v=(float *)malloc((unsigned) (nh-nl+1)*sizeof(float));
	if (!v)
		nrerror("allocation failure in vector()");
	return v-nl;
}

int *ivector(nl,nh)
int nl,nh;
{
	int *v;

	v=(int *)malloc((unsigned) (nh-nl+1)*sizeof(int));
	if (!v)
		nrerror("allocation failure in ivector()");
	return v-nl;
}

double *dvector(int nl,int nh)
{
	double *v;

	v=(double *)malloc((unsigned) (nh-nl+1)*sizeof(double));
	if (!v)
		nrerror("allocation failure in dvector()");
	return v-nl;
}

float **matrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	float **m;

	m = (float **) malloc((unsigned) (nrh-nrl+1)*sizeof(float*));
	if (!m)
		nrerror("allocation failure 1 in matrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i] = (float *) malloc((unsigned) (nch-ncl+1)*sizeof(float));
		if (!m[i])
			nrerror("allocation failure 2 in matrix()");
		m[i] -= ncl;
	}
	return m;
}

double **dmatrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	double **m;

	m = (double **) malloc((unsigned) (nrh-nrl+1)*sizeof(double*));
	if (!m)
		nrerror("allocation failure 1 in dmatrix()");
	m -= nrl;

	for(i = nrl;i<=nrh; i++) {
		m[i] = (double *) malloc((unsigned) (nch-ncl+1)*sizeof(double));
		if (!m[i])
			nrerror("allocation failure 2 in dmatrix()");
		m[i] -= ncl;
	}
	return m;
}

int **imatrix(int nrl,int nrh,int ncl,int nch)
{
	int i,**m;

	m = (int **)malloc((unsigned) (nrh-nrl+1)*sizeof(int*));
	if (!m)
		nrerror("allocation failure 1 in imatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i] = (int *)malloc((unsigned) (nch-ncl+1)*sizeof(int));
		if (!m[i])
			nrerror("allocation failure 2 in imatrix()");
		m[i] -= ncl;
	}
	return m;
}



float **submatrix(a,oldrl,oldrh,oldcl,oldch,newrl,newcl)
float **a;
int oldrl,oldrh,oldcl,oldch,newrl,newcl;
{
	int i,j;
	float **m;

	m=(float **) malloc((unsigned) (oldrh-oldrl+1)*sizeof(float*));
	if (!m)
		nrerror("allocation failure in submatrix()");
	m -= newrl;

	for(i=oldrl,j=newrl;i<=oldrh;i++,j++)
		m[j]=a[i]+oldcl-newcl;

	return m;
}



void free_vector(float *v,int nl,int nh)
{
	free((char*) (v+nl));
}

void free_ivector(int *v,int nl,int nh)
{
	free((char*) (v+nl));
}

void free_dvector(double *v,int nl,int nh)
{
	free((char*) (v+nl));
}



void free_matrix(float **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void free_dmatrix(double **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void free_imatrix(int **m,int nrl,int nrh,int ncl,int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}



void free_submatrix(float **b,int nrl,int nrh,int ncl,int nch)
{
	free((char*) (b+nrl));
}



float **convert_matrix(float *a,int nrl,int nrh,int ncl,int nch)
{
	int i,j,nrow,ncol;
	float **m;

	nrow=nrh-nrl+1;
	ncol=nch-ncl+1;
	m = (float **) malloc((unsigned) (nrow)*sizeof(float*));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m -= nrl;
	for(i=0,j=nrl;i<=nrow-1;i++,j++) m[j]=a+ncol*i-ncl;
	return m;
}



void free_convert_matrix(float **b,int nrl,int nrh,int ncl,int nch)
{
	free((char*) (b+nrl));
}



void surv2enu(double dist, double azd, double azm, double azs,
		double eld, double elm, double els,
		double *eastp, double *northp, double *upp)
{
	double azrad;
	double elrad;
	double pi = 3.14159265358979;

	azrad = (azd + azm/60.0 + azs/3600.0)*pi/180.0;
	elrad = (eld + elm/60.0 + els/3600.0)*pi/180.0;

	*eastp = sin(azrad)*dist*cos(elrad);
	*northp = cos(azrad)*dist*cos(elrad);
	*upp = dist*sin(elrad);

}


/* Construct principal axis rotation matrix */
void make_rot_dmat(double theta, int axis, dmat *Rp) {

	double cth = cos(theta),sth=sin(theta);
	int nr, nc;

	if (!dmatinitialized(Rp))
		initdmat(Rp, 1, 3, 1, 3);

	nr = Rp->nrh - Rp->nrl + 1;
	nc = Rp->nch - Rp->ncl + 1;

	if ((nr != 3) || (nc != 3))
		nrerror("Matrix initialized, but wrong size in make_rot_dmat");

	zerodmat(Rp);
	switch (axis) {


	case 1:   /* x axis rotation */
		setdmat(Rp, Rp->nrl, Rp->ncl, 1.0);
		setdmat(Rp, Rp->nrl+1, Rp->ncl+1, cth);
		setdmat(Rp, Rp->nrl+2, Rp->ncl+2, cth);
		setdmat(Rp, Rp->nrl+1, Rp->ncl+2, sth);
		setdmat(Rp, Rp->nrl+2, Rp->ncl+1, -sth);

		break;

	case 2:  /* y axis rotation */
		setdmat(Rp, Rp->nrl, Rp->ncl, cth);
		setdmat(Rp, Rp->nrl+1, Rp->ncl+1, 1.0);
		setdmat(Rp, Rp->nrl+2, Rp->ncl+2, cth);
		setdmat(Rp, Rp->nrl, Rp->ncl+2, -sth);
		setdmat(Rp, Rp->nrl+2, Rp->ncl, sth);

		break;

	case 3: /* z axis rotation */
		setdmat(Rp, Rp->nrl+2, Rp->ncl+2, 1.0);
		setdmat(Rp, Rp->nrl+1, Rp->ncl+1, cth);
		setdmat(Rp, Rp->nrl, Rp->ncl, cth);
		setdmat(Rp, Rp->nrl, Rp->ncl+1, sth);
		setdmat(Rp, Rp->nrl+1, Rp->ncl, -sth);

		break;
	}

}


/* LU Decomposition/backsubstitution routines */

#define TINY 1.0e-20

void dludcmp (double **a, int n, int *indx, double *d)

/*
 * LU Decomposition - Numerical Recipes in C: Press W. H., Flannery B. P.,
 *                    Toukolsky S. A., Vetterling W. T.
 *
 *   Given an n x n matrix a[1..n][1..], this routine replaces it by the LU
 *   decomposition of a rowwise permutation of itself. 'a' and 'n' are
 *   input.
 *   At the end the soln is output in 'a'. indx[1..n] is the output vector
 *   which records the row permutation effected by the partial pivoting;
 *   'd' is output as either +1 or -1 depending on whether the number of
 *   row changes was even or odd, respectively. This routine is used in
 *   combination with 'lubksb' to solve linear eqn.s or invert a matrix.
 *
 *   Awele Ndili @ October 1991
 */

{
	int     i,
	imax,
	j,
	k;
	double   big,
	dum,
	sum,
	temp;
	double  *vv,
	*dvector ();		/* vv stores implicit scaling of each row
	*/
	void nrerror (), free_dvector ();

	vv = dvector (1, n);
	*d = 1.0;			/* No row interchanges yet  */
	for (i = 1; i <= n; i++) {	/* Loop over rows to get implicit scaling
	 */
		big = 0.0;		/* information */
		for (j = 1; j <= n; j++)
			if ((temp = fabs (a[i][j])) > big)
				big = temp;
		if (big == 0.0)
			nrerror ("Singular Matrix in routine LUDCMP");

		/* No nonzero largest element */
		vv[i] = 1.0 / big;	/* Save the scaling       */
	}

	for (j = 1; j <= n; j++) {
		for (i = 1; i < j; i++) {
			sum = a[i][j];
			for (k = 1; k < i; k++)
				sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
		}
		big = 0.0;		/* Initialize the search for the largest
			   pivot element */
		for (i = j; i <= n; i++) {
			sum = a[i][j];
			for (k = 1; k < j; k++)
				sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
			if ((dum = vv[i] * fabs (sum)) >= big) {
				/* Is the figure of merit for the */
				big = dum;	/* pivot better than the best so far? */
				imax = i;
			}
		}
		if (j != imax) {	/* Do we need to interchange rows ? */
			for (k = 1; k <= n; k++) {/* Yes do it */
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			*d = -(*d);		/* Change also the sign of d.
			*/
			vv[imax] = vv[j];	/* Also interchange the scale factor */
		}

		indx[j] = imax;
		if (a[j][j] == 0.0)
			a[j][j] = TINY;
		/*  If the pivot element is zero the matrix is singular (at least
		 *  to the precision of the algorithm). For some applications on
		 *  singular matrices, it is desireable to substitute TINY for zero
		 */

		if (j != n) {		/* Now, finally, divide the pivot element
		 */
			dum = 1.0 / (a[j][j]);
			for (i = j + 1; i <= n; i++)
				a[i][j] *= dum;
		}
	}				/* Go back to next column in the reduction
	 */
	free_dvector (vv, 1, n);
}


void dlubksb(double **a, int n, int *indx, double b[])

/*
 * Forward / Backward Solver - Numerical Recipes in C: Press W. H., Flannery
 * B. P.,Toukolsky S. A., Vetterling W. T.
 *
 *
 * Solves the set of n linear equations Ax = B. Here a[1..n][1..n] is input, not
 * as the matrix A, but as its LU decomposition, determined by the routine
 * ludcmp. b[1..n] is input as the right-hand side vector B, and returns with
 * the solution vector x. a, n, and indx are not modified by this routine and
 * can be left in place for successive calls with different right-hand sides
 * b. This routine takes into account the possibility that b will begin with
 * many zero elements, so it is effecient for use in matrix inversion.
 *
 * Awele Ndili @ October 1991
 */

{
	int             i, ii = 0, ip, j;
	double           sum;

	/*
	 * When ii is set to a positive value it will become the index of the
	 * first nonvanishing element of b. We now do the forward
	 * substitution ...
	 */
	for (i = 1; i <= n; i++) {
		ip = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if (ii)
			for (j = ii; j <= i - 1; j++)
				sum -= a[i][j] * b[j];
		else if (sum)
			ii = i;	/* A nonzero element was encountered, */
		b[i] = sum;	/* so from now on we have */
		/* to do the sums in the loop above */
	}
	for (i = n; i >= 1; i--) {	/* Now we do the back substitution  */
		sum = b[i];
		for (j = i + 1; j <= n; j++)
			sum -= a[i][j] * b[j];
		b[i] = sum / a[i][i];	/* Store a component of the */
	}			/* solution vector X. */
	/* All Done !!! */
}




/*  cholesky.c -- Cholesky Factorization and Determinant		*/
/*  For use with Symmetric, Positive Definite Matrices Only!!!		*/
/*  Copyright (C) 1992  Trimble Navigation  all rights reserved  	*/
/*  Author:  Jeff Crerie						*/

/*  Modified:  Clark Cohen (adapted vectors to numerical recipes form,	*/
/*  reference only lower triangle of a, and removed dependence on p	*/
/*  by storing L on top of A)						*/

/*  Ref. J.H. Wilkinson and C. Reinsch, ed., Handbook for Automatic  	*/
/*  Computation, v.II Linear Algebra, 1971, pp 1-30  			*/

/*  returns determinant of a or -1 if a is not positive definite 	*/

double dcholfact(double **A,int n)

{
	double	x, d;
	int		i, j, k;

	d = 1.0;

	for (j=1; j<=n; j++)  {
		for (i=j; i<= n; i++)  {
			x = A[i][j];
			for (k=1; k<j; k++) x -= A[j][k]*A[i][k];
			if (j == i)  {
				d *= x;
				if (x <= 0.0)
				{
					nrerror("dcholfact error");
					return -1;  /* flag error */
				}
				A[j][j] = sqrt(x);
			}	/* end if (j == i) */
			else    A[i][j] = x/A[j][j];
		}	/* end i */
	}	/* end j */

	return d;

}			/* end cholfact() */



/*  GPS Core -- Cholesky Solution to Ax = b				*/
/*  For use with Symmetric, Positive Definite Matrices Only!!!		*/
/*  Copyright (C) 1992	Trimble Navigation  all rights reserved 	*/
/*  Author:  Jeff Crerie						*/

/*  Modified:  Clark Cohen (adapted vectors to numerical recipes form,	*/
/*  reference only lower triangle of a, and removed dependence on p	*/
/*  by storing L on top of A)						*/

/*  Ref. J.H. Wilkinson and C. Reinsch, ed., Handbook for Automatic	*/
/*  Computation, v.II Linear Algebra, 1971, pp 1-30			*/


/* Solution of Lx = b					*/

void dbacksubL(double **L,double *x,double *b,int n)

{
	double	z;
	int	i, k;

	for (i = 1; i <= n; i++)  {
		z = b[i];
		for (k=1; k<i; k++) z -= L[i][k]*x[k];
		x[i] = z/L[i][i];
	}

}



void dcholbksb(double **L,double *x,double *b,int n)

{

	/* Solution of Ly = b... */
	dbacksubL(L,x,b,n);

	/* Solution of LTx = y... */
	dbacksubLT(L,x,x,n);

}			/* end cholbksb() */



/* double precision solution of LTx = b */

int dbacksubLT(double **L,double *x,double *b,int n)

{
	double	z;
	int	i, k;

	for (i=n; i>=1; i--)  {
		z = b[i];
		for (k=i+1; k<=n; k++) z -= L[k][i]*x[k];
		if (L[i][i] == 0.0) return -1;  /* singular matrix */
		x[i] = z/L[i][i];
	}
	return 0;
}



void dget_Att_Rot( double **A, double azi, double pit, double rol ) {


	A[1][1] = cos(pit)*sin(azi);

	A[1][2] = cos(rol)*cos(azi)+sin(rol)*sin(pit)*sin(azi);

	A[1][3] = -sin(rol)*cos(azi)+cos(rol)*sin(pit)*sin(azi);

	A[2][1] = cos(pit)*cos(azi);

	A[2][2] = -cos(rol)*sin(azi)+sin(rol)*sin(pit)*cos(azi);

	A[2][3] = sin(rol)*sin(azi)+cos(rol)*sin(pit)*cos(azi);

	A[3][1] = sin(pit);

	A[3][2] = -sin(rol)*cos(pit);

	A[3][3] = -cos(rol)*cos(pit);

}



/* New matrix library */

void initdmat(dmat *dmatp, int nrl, int nrh, int ncl, int nch)
{

	/* Make sure not initialized */

	if (dmatp->mat != (double **) 0)
		nrerror("Matrix already initialized.\n");
	dmatp->mat = dmatrix(nrl, nrh, ncl, nch);
	dmatp->nrl = nrl;
	dmatp->nrh = nrh;
	dmatp->ncl = ncl;
	dmatp->nch = nch;

}


void freedmat(dmat *dmatp)
{

	if (dmatp->mat != (double **) 0)
		free_dmatrix(dmatp->mat, dmatp->nrl, dmatp->nrh, dmatp->ncl, dmatp->nch);

	dmatp->mat = (double **) 0;
	dmatp->nrl = 0;
	dmatp->nrh = 0;
	dmatp->ncl = 0;
	dmatp->nch = 0;

}

void initdvec(dvec *dvecp, int nel, int neh)
{
	if (dvecp->vec != (double *) 0)
		nrerror("Vector already initialized in initdvec.\n");

	dvecp->vec = dvector(nel, neh);
	dvecp->nel = nel;
	dvecp->neh = neh;

}

void freedvec(dvec *dvecp)
{
	if (dvecp->vec != (double *) 0)
		free_dvector(dvecp->vec, dvecp->nel, dvecp->neh);
	dvecp->vec = (double *) 0;
	dvecp->nel = 0;
	dvecp->neh = 0;

}

void initivec(ivec *dvecp, int nel, int neh)
{
	if (dvecp->vec != (int *) 0)
		nrerror("Vector already initialized in initivec.\n");

	dvecp->vec = ivector(nel, neh);
	dvecp->nel = nel;
	dvecp->neh = neh;

}

void freeivec(ivec *dvecp)
{
	if (dvecp->vec != (int *) 0)
		free_ivector(dvecp->vec, dvecp->nel, dvecp->neh);
	dvecp->vec = (int *) 0;
	dvecp->nel = 0;
	dvecp->neh = 0;

}


/* Calculate the matrix product of mat1 (nr1 x nc1) and mat2(nc1 x  nc2) and
     puts the result in user provided workspace ans (1 .. nr1 x 1 .. nc2) */
void dmatxdmat(dmat *mat1p, dmat *mat2p, dmat *ansp) {

	int i,j,k,nr1, nc1, nr2, nc2, nrl1, ncl1, nrh1, nch1, nrl2, ncl2, nrh2, nch2, nrlans, nrhans, nclans, nchans, nrans, ncans;

	if (mat1p->mat == (double **) 0)
		nrerror("mat1 not initialized\n");

	if (mat2p->mat == (double **) 0)
		nrerror("mat2 not initialized\n");


	/* Check to make sure matrices are correct size */
	nrl1 = mat1p->nrl;
	nrh1 = mat1p->nrh;
	ncl1 = mat1p->ncl;
	nch1 = mat1p->nch;
	nr1 = nrh1-nrl1+1;
	nc1 = nch1-ncl1+1;


	nrl2 = mat2p->nrl;
	nrh2 = mat2p->nrh;
	ncl2 = mat2p->ncl;
	nch2 = mat2p->nch;
	nr2 = nrh2-nrl2+1;
	nc2 = nch2-ncl2+1;

	if (nc1 != nr2)
		nrerror("Cannot multiply mat1 by mat2 (wrong dimensions)");

	/* Create memory for ans, if not already done */
	if (ansp->mat == (double **) 0)
		initdmat(ansp, 1, nr1, 1, nc2);

	nrlans = ansp->nrl;
	nrhans = ansp->nrh;
	nclans = ansp->ncl;
	nchans = ansp->nch;
	nrans = nrhans-nrlans+1;
	ncans = nchans-nclans+1;

	if ((nrans != nr1) || (ncans != nc2))
		nrerror("Memory for answer allocated, but wrong dimension");

	for (i=1;i<=nr1;i++)
		for (j=1;j<=nc2;j++) {
			ansp->mat[nrlans+i-1][nclans+j-1] = 0.0;
			for (k=0;k<=nc1-1;k++)
				ansp->mat[nrlans+i-1][nclans+j-1] += mat1p->mat[i+nrl1-1][k+ncl1]*mat2p->mat[k+nrl2][j+ncl2-1];
		}

}

void dmatxdmatsim(dmat *mat1p, dmat *mat2p, dmat *ansp) {

	int i,j,k,nr1, nc1, nr2, nc2, nrl1, ncl1, nrh1, nch1, nrl2, ncl2, nrh2, nch2, nrlans, nrhans, nclans, nchans, nrans, ncans;

	if (mat1p->mat == (double **) 0)
		nrerror("mat1 not initialized\n");

	if (mat2p->mat == (double **) 0)
		nrerror("mat2 not initialized\n");


	/* Check to make sure matrices are correct size */
	nrl1 = mat1p->nrl;
	nrh1 = mat1p->nrh;
	ncl1 = mat1p->ncl;
	nch1 = mat1p->nch;
	nr1 = nrh1-nrl1+1;
	nc1 = nch1-ncl1+1;


	nrl2 = mat2p->nrl;
	nrh2 = mat2p->nrh;
	ncl2 = mat2p->ncl;
	nch2 = mat2p->nch;
	nr2 = nrh2-nrl2+1;
	nc2 = nch2-ncl2+1;

	if (nc1 != nr2)
		nrerror("Cannot multiply mat1 by mat2 (wrong dimensions)");

	/* Create memory for ans, if not already done */
	if (ansp->mat == (double **) 0)
		initdmat(ansp, 1, nr1, 1, nc2);

	nrlans = ansp->nrl;
	nrhans = ansp->nrh;
	nclans = ansp->ncl;
	nchans = ansp->nch;
	nrans = nrhans-nrlans+1;
	ncans = nchans-nclans+1;

	if ((nrans != nr1) || (ncans != nc2))
		nrerror("Memory for answer allocated, but wrong dimension");

	for (i=1;i<=nr1;i++)
		for (j=i;j<=nc2;j++) {
			ansp->mat[nrlans+i-1][nclans+j-1] = 0.0;
			for (k=0;k<=nc1-1;k++)
				ansp->mat[nrlans+i-1][nclans+j-1] += mat1p->mat[i+nrl1-1][k+ncl1]*mat2p->mat[k+nrl2][j+ncl2-1];
			ansp->mat[nrlans+j-1][nclans+i-1] = ansp->mat[nrlans+i-1][nclans+j-1];
		}

}



/* Calculate the matrix sum of mat1 (nr1 x nc1) and mat2(nnr1 x  nc1) and
     puts the result in user provided workspace ans (1 .. nr1 x 1 .. nc1) */
void dmatplsdmat(dmat *mat1p, dmat *mat2p, dmat *ansp) {

	int i,j,nr1, nc1, nr2, nc2, nrl1, ncl1, nrh1, nch1, nrl2, ncl2, nrh2, nch2, nrlans, nrhans, nclans, nchans, nrans, ncans;

	if (mat1p->mat == (double **) 0)
		nrerror("mat1 not initialized\n");

	if (mat2p->mat == (double **) 0)
		nrerror("mat2 not initialized\n");



	/* Check to make sure matrices are correct size */
	nrl1 = mat1p->nrl;
	nrh1 = mat1p->nrh;
	ncl1 = mat1p->ncl;
	nch1 = mat1p->nch;
	nr1 = nrh1-nrl1+1;
	nc1 = nch1-ncl1+1;


	nrl2 = mat2p->nrl;
	nrh2 = mat2p->nrh;
	ncl2 = mat2p->ncl;
	nch2 = mat2p->nch;
	nr2 = nrh2-nrl2+1;
	nc2 = nch2-ncl2+1;

	if ((nr1 != nr2)||(nc1 != nc2))
		nrerror("Cannot add mat1 to mat2 (wrong dimensions)");


	/* Create memory for ans, if not already done */
	if (ansp->mat == (double **) 0)
		initdmat(ansp, 1, nr1, 1, nc1);

	nrlans = ansp->nrl;
	nrhans = ansp->nrh;
	nclans = ansp->ncl;
	nchans = ansp->nch;
	nrans = nrhans-nrlans+1;
	ncans = nchans-nclans+1;

	if ((nrans != nr1) || (ncans != nc1))
		nrerror("Memory for answer allocated, but wrong dimension");

	for (i=1;i<=nr1;i++)
		for (j=1;j<=nc1;j++)
			ansp->mat[nrlans+i-1][nclans+j-1] = mat1p->mat[i+nrl1-1][j+ncl1-1] + mat2p->mat[i+nrl2-1][j+ncl2-1];

}


int dmatinitialized(dmat *dmatp) {

	if (dmatp->mat != (double **) 0)
		return 1;
	else
		return 0;
}

int dvecinitialized(dvec *dvecp) {

	if (dvecp->vec != (double *) 0)
		return 1;
	else
		return 0;
}

int ivecinitialized(ivec *dvecp) {

	if (dvecp->vec != (int *) 0)
		return 1;
	else
		return 0;
}


void dmatxdvec(dmat *matp, dvec *vecp, dvec *ansp) {

	int i,j,nr,nc,nel;

	if (!dmatinitialized(matp))
		nrerror("Matrix not initialized in dmatxdvec\n");
	if (!dvecinitialized(vecp))
		nrerror("Vector not initialized in dmatxdvec\n");




	nr = matp->nrh-matp->nrl+1;
	nc = matp->nch-matp->ncl+1;
	nel = vecp->neh-vecp->nel+1;

	if (nc != nel)
		nrerror("Incompatible dimensions in dmatxdvec");

	if (!dvecinitialized(ansp))
		initdvec(ansp,1, nr);

	if ((ansp->neh-ansp->nel+1) != nr)
		nrerror("Memory for vector initialized, but wrong dimension in dmatxdvec\n");



	for (i=1;i<=nr;i++){
		ansp->vec[i+ansp->nel-1] = 0.0;
		for (j=1;j<=nc;j++)
			ansp->vec[i+ansp->nel-1] += matp->mat[matp->nrl+i-1][matp->ncl+j-1]*vecp->vec[vecp->nel+j-1];
	}


}


void setdmat(dmat *dmatp, int sub1, int sub2, double val)
{

	if (!dmatinitialized(dmatp))
		nrerror("Matrix not initiallized in setdmat\n");

	if ((sub1 > dmatp->nrh) || (sub1 < dmatp->nrl))
		nrerror("Bad row index in setdmat\n");

	if ((sub2 > dmatp->nch) || (sub2 < dmatp->ncl))
		nrerror("Bad column index in setdmat\n");

	dmatp->mat[sub1][sub2] = val;

	return;
}

void setdvec(dvec *dvecp, int sub, double val)
{
	if (!dvecinitialized(dvecp))
		nrerror("Vector not initiallized in setdvec\n");

	if ((sub > dvecp->neh) || (sub < dvecp->nel))
		nrerror("Bad index in setdvec\n");
	dvecp->vec[sub] = val;
}

double getdvec(dvec *dvecp, int sub)
{

	if (!dvecinitialized(dvecp))
		nrerror("Vector not initialized in getdvec\n");

	if ((sub > dvecp->neh) || (sub < dvecp->nel))
		nrerror("Bad index in getdvec\n");

	return dvecp->vec[sub];

}

int getivec(ivec *ivecp, int sub)
{

	if (!ivecinitialized(ivecp))
		nrerror("Vector not initialized in getivec\n");


	if ((sub > ivecp->neh) || (sub < ivecp->nel))
	{

		/*    printivec(ivecp);*/
#ifdef TAIR
		wprintf(winKLS, "Bad index = %d\n",sub);
#endif

		nrerror("Bad index in getivec\n");
	}

	return ivecp->vec[sub];

}

void setivec(ivec *dvecp, int sub, int val)
{

	if (!ivecinitialized(dvecp))
		nrerror("Vector not initialized in setivec\n");

	if ((sub > dvecp->neh) || (sub < dvecp->nel))
		nrerror("Bad index in setivec\n");

	dvecp->vec[sub] = val;

}

void transdmat(dmat *matp, dmat *ansp) {

	int i,j,nr,nc;

	if (!dmatinitialized(matp))
		nrerror("Matrix not initialized in transdmat\n");



	nr = matp->nrh-matp->nrl+1;
	nc = matp->nch-matp->ncl+1;

	if (!dmatinitialized(ansp))
		initdmat(ansp, 1, nc, 1, nr);
	else
		if ((ansp->nrh-ansp->nrl+1 != nc) || (ansp->nch-ansp->ncl+1 != nr))
			nrerror("Matrix initialized, but wrong dimension in transdmat");

	for (i=1;i<=nr;i++)
		for (j=1;j<=nc;j++)
			ansp->mat[ansp->nrl-1+j][ansp->ncl-1+i]=matp->mat[matp->nrl+i-1][matp->ncl+j-1];

	return;
}

/* calculates inv(ATA) where ATA is a positive definite, symmetric matrix*/
void dmatsinv(dmat *ATAp, dmat *Asip) {
	/* ATA: matrix to be inverted.  */
	/* Asi: Workspace for inverse of ATA */

	double *colj, *eyej;
	int i,j,nr,nc;
	dmat S = ZERODMAT;

	if (!dmatinitialized(ATAp))
		nrerror("Matrix not initialized in dmatsinv\n");
	if ((ATAp->nrl != 1) || (ATAp->ncl != 1))
		nrerror("Indices must start at one in dmatsinv\n");


	nr = ATAp->nrh-ATAp->nrl+1;
	nc = ATAp->nch-ATAp->ncl+1;

	if (nr != nc)
		nrerror("Matrix not square in dmatsinv\n");

	if (!dmatinitialized(Asip))
		initdmat(Asip, 1, nr, 1, nc);
	else
		if ((Asip->nrh-Asip->nrl+1 != nr) || (Asip->nch-Asip->ncl+1 != nc))
			nrerror("Matrix initialized, but wrong dimension in dmatsinv\n");

	colj = dvector(1, nc);
	eyej = dvector(0, nc);

	copydmat(ATAp, &S);
	if (dcholfact(S.mat, nc) < 0.0)
	{
//		printdmat(ATAp);
		nrerror("dcholfact error in dmatsinv\n");
	}

	for (i=1;i<=nc;i++) eyej[i] = 0.0;
	for (j=1;j<=nc;j++) {
		eyej[j] = 1.0;
		eyej[j-1] = 0.0;
		dcholbksb(S.mat, colj, eyej, nc);
		for (i=1;i<=nc;i++)
			setdmat(Asip, i, j, colj[i]);
	}

	free_dvector(colj, 1, nc);
	free_dvector(eyej, 0, nc);
	freedmat(&S);
}

/* calculates inv(ATA) */
void dmatinv(dmat *ATAp, dmat *Asip) {
	/* ATA: matrix to be inverted.  */
	/* Asi: Workspace for inverse of ATA */

	double *eyej, d;
	int i,j,nr,nc,*indx;


	if (!dmatinitialized(ATAp))
		nrerror("Matrix not initialized in dmatsinv\n");
	if ((ATAp->nrl != 1) || (ATAp->ncl != 1))
		nrerror("Indices must start at one in dmatsinv\n");


	nr = ATAp->nrh-ATAp->nrl+1;
	nc = ATAp->nch-ATAp->ncl+1;

	if (nr != nc)
		nrerror("Matrix not square in dmatsinv\n");

	if (!dmatinitialized(Asip))
		initdmat(Asip, 1, nr, 1, nc);
	else
		if ((Asip->nrh-Asip->nrl+1 != nr) || (Asip->nch-Asip->ncl+1 != nc))
			nrerror("Matrix initialized, but wrong dimension in dmatsinv\n");

	eyej = dvector(0, nc);
	indx = ivector(1,nc);

	dludcmp(ATAp->mat, nc, indx, &d);


	for (i=1;i<=nc;i++) eyej[i] = 0.0;

	for (j=1;j<=nc;j++) {
		eyej[j] = 1.0;
		dlubksb(ATAp->mat, nc, indx, eyej);
		for (i=1;i<=nc;i++) {
			setdmat(Asip, i, j, eyej[i]);
			eyej[i] = 0.0;
		}
	}

	free_dvector(eyej, 0, nc);
	free_ivector(indx, 1, nc);
}


void dvecxscal(dvec *dvecp, double scal, dvec *ansp)
{
	int i,len;

	if (!dvecinitialized(dvecp))
		nrerror("Vector not initialized in dvecxscal");

	len = dvecp->neh - dvecp->nel + 1;

	if (!dvecinitialized(ansp))
		initdvec(ansp, 1, len);
	else
		if (ansp->neh - ansp->nel + 1 != len)
			nrerror("Vector initialized, but wrong dimension in dvecxscal\n");

	for (i=1;i<=len;i++)
		ansp->vec[i-1+ansp->nel] = scal*dvecp->vec[i-1+dvecp->nel];


	return;

}

void dmatxscal(dmat *dmatp, double scal, dmat *ansp)
{
	int i,j,nr,nc;


	if (!dmatinitialized(dmatp))
		nrerror("Matrix not initialized in dmatxscal");

	nr = dmatp->nrh-dmatp->nrl+1;
	nc = dmatp->nch-dmatp->ncl+1;

	if (!dmatinitialized(ansp))
		initdmat(ansp, 1, nr, 1, nc);
	else
		if ((ansp->nrh-ansp->nrl+1 != nr) || (ansp->nch-ansp->ncl+1 != nc))
			nrerror("Matrix initialized, but wrong dimension in dmatxscal\n");

	for (i=1;i<=nr;i++)
		for (j=1;j<=nc;j++)
			setdmat(ansp,i+ansp->nrl-1,j+ansp->ncl-1, scal * dmatp->mat[i+dmatp->nrl-1][j+dmatp->ncl-1]);

	return;
}

void dmateye(dmat *dmatp, int n)
{
	int i,j,nr,nc;


	if (!dmatinitialized(dmatp))
		initdmat(dmatp, 1, n, 1, n);

	nr = dmatp->nrh-dmatp->nrl+1;
	nc = dmatp->nch-dmatp->ncl+1;

	if ((nr != nc) || (nr != n))
		nrerror("Matrix initialized, but wrong dimension in dmateye\n");

	for (i=1;i<=n;i++)
		for (j=1;j<=n;j++)
			setdmat(dmatp, i+dmatp->nrl-1,j+dmatp->ncl-1, 0.0);

	for (j=1;j<=n;j++)
		setdmat(dmatp, j+dmatp->nrl-1,j+dmatp->ncl-1, 1.0);

	return;

}


void subdmat(dmat *dmatp, int row1, int row2, int col1, int col2, dmat *ansp)
{
	int i, j, nrow,ncol;

	if (!dmatinitialized(dmatp))
		nrerror("Matrix not initialized in subdmat\n");

	if ((row1 < dmatp->nrl) || (row1 > dmatp->nrh) ||(row2 < dmatp->nrl) || (row2 > dmatp->nrh))
		nrerror("Bad row index in subdmat\n");

	if ((col1 < dmatp->ncl) || (col1 > dmatp->nch) ||(col2 < dmatp->ncl) || (col2 > dmatp->nch))
		nrerror("Bad column index in subdmat\n");

	nrow = row2-row1+1;
	ncol = col2-col1+1;

	if (!dmatinitialized(ansp))
		initdmat(ansp,1,nrow,1,ncol);
	else
	{
		if ((ansp->nch - ansp->ncl +1) != ncol)
			nrerror("Matrix initialized, but wrong dimension in subdmat");
		if ((ansp->nrh - ansp->nrl +1) != nrow)
			nrerror("Matrix initialized, but wrong dimension in subdmat");
	}
	for (i=0;i<=nrow-1;i++)
		for (j=0;j<=ncol-1;j++)
			setdmat(ansp, ansp->nrl+i, ansp->ncl+j, dmatp->mat[row1+i][col1+j]);

	return;

}




void subdvec(dvec *dvecp, int el1, int el2, dvec *ansp)
{
	int i,nsub;

	if (!dvecinitialized(dvecp))
		nrerror("Vector not initialized in subdvec\n");

	if ((el1 < dvecp->nel) || (el1 > dvecp->neh) ||(el2 < dvecp->nel) || (el2 > dvecp->neh))
		nrerror("Bad index in subdvec\n");

	nsub = el2-el1+1;

	if (!dvecinitialized(ansp))
		initdvec(ansp,1,nsub);
	else
		if ((ansp->neh - ansp->nel +1) != nsub)
			nrerror("Vector initialized, but wrong dimension in subdvec");

	for (i=1;i<=nsub;i++)
		setdvec(ansp, ansp->nel+i-1, dvecp->vec[el1+i-1]);


	return;

}

void dvecplsdvec(dvec *dvec1p, dvec *dvec2p, dvec *ansp)
{
	int i, len;

	if ((!dvecinitialized(dvec1p)) ||(!dvecinitialized(dvec2p)))
		nrerror("Vector not initialized in dvecplsdvec\n");

	len = dvec1p->neh - dvec1p->nel + 1;

	if ((dvec2p->neh - dvec2p->nel + 1) != len)
	{
//		printdvec(dvec1p);
//		printdvec(dvec2p);
		nrerror("Vector dimensions must agree in dvecplsdvec\n");
	}
	if (!dvecinitialized(ansp))
		initdvec(ansp,1,len);
	else
		if ((ansp->neh - ansp->nel + 1) != len)
			nrerror("Memory allocated, but wrong dimension in dvecplsdvec\n");

	for (i=0;i<=len-1;i++)
		setdvec(ansp, i+ansp->nel, dvec1p->vec[dvec1p->nel+i]+dvec2p->vec[dvec2p->nel+i]);

	return;
}

double dvecdotdvec(dvec *dvec1p, dvec *dvec2p)
{
	int i, len;
	double dot=0.0;

	if ((!dvecinitialized(dvec1p)) ||(!dvecinitialized(dvec2p)))
		nrerror("Vector not initialized in dvecdotdvec\n");

	len = dvec1p->neh - dvec1p->nel + 1;

	if ((dvec2p->neh - dvec2p->nel + 1) != len)
		nrerror("Vector dimensions must agree in dvecdotdvec\n");


	for (i=0;i<=len-1;i++)
		dot += dvec1p->vec[dvec1p->nel+i]*dvec2p->vec[dvec2p->nel+i];

	return dot;
}

double dvec2norm(dvec *vecp)
{
	double n;

	n=sqrt(dvecdotdvec(vecp,vecp));

	return n;
}



void zerodmat(dmat *dmatp)
{
	int i,j;

	if (!dmatinitialized(dmatp))
		nrerror("Matrix not initialized in zerodmat\n");

	for (i=dmatp->nrl;i<=dmatp->nrh;i++)
		for (j=dmatp->ncl;j<=dmatp->nch;j++)
			setdmat(dmatp,i,j,0.0);

	return;
}

void zerodvec(dvec *dvecp)
{
	int i;

	if (!dvecinitialized(dvecp))
		nrerror("Vector not initialized in zerodvec\n");

	for (i=dvecp->nel;i<=dvecp->neh;i++)
		setdvec(dvecp, i, 0.0);

	return;
}

void zeroivec(ivec *ivecp)
{
	int i;

	if (!ivecinitialized(ivecp))
		nrerror("Vector not initialized in zeroivec\n");

	for (i=ivecp->nel;i<=ivecp->neh;i++)
		setivec(ivecp, i, 0);

	return;
}

/* Given an observation matrix (*Hp), the first n of whose states
   need not be estimated, return the transformation matrix *Tp
   that eliminates those states from the problem, and the
   corresponding reduced observation matrix (*Hrp) */

/* This should probably be done with gauss elimination, but for
   now, here's a hack */

void reduce_obsv(dmat *Hp, dmat *Hrp, dmat *Tp, int n)
{
	int nrow,ncol;
	dmat Ha = ZERODMAT;
	dmat Hai = ZERODMAT;
	dmat scratchmat1 = ZERODMAT;




	if (!dmatinitialized(Hp))
		nrerror("Matrix not initialized in reduce_obsv\n");

	ncol = Hp->nch - Hp->ncl + 1;
	nrow = Hp->nrh - Hp->nrl + 1;

	if ((n > ncol)||(n > nrow))
		nrerror("n too large in reduce_obsv\n");


	if (nrow >= ncol)
		nrerror("Wrong size observation matrix for this implementation of reduce_obsv\n");

	if (!dmatinitialized(Hrp))
		initdmat(Hrp,1,nrow-n,1,ncol-n);
	else
		if ((Hrp->nrh-Hrp->nrl+1 != nrow-n)||(Hrp->nch-Hrp->ncl+1 != ncol-n))
			nrerror("Hrp initialized but wrong dimension in reduce_obsv\n");

	if (!dmatinitialized(Tp))
		initdmat(Tp,1,nrow-n,1,nrow);
	else
		if ((Tp->nrh-Tp->nrl+1 != nrow-n)||(Tp->nch-Tp->ncl+1 != nrow))
			nrerror("Tp initialized but wrong dimension in reduce_obsv\n");
	subdmat(Hp, Hp->nrl, Hp->nrh, Hp->ncl, Hp->ncl+nrow-1, &Ha);
	dmatinv(&Ha, &Hai);
	dmatxdmat(&Hai, Hp, &scratchmat1);
	subdmat(&scratchmat1, scratchmat1.nrl+n, scratchmat1.nrh, scratchmat1.ncl+n,scratchmat1.nch,Hrp);
	subdmat(&Hai, Hai.nrl+n, Hai.nrh, Hai.ncl, Hai.nch, Tp);

	freedmat (&Ha);
	freedmat (&Hai);
	freedmat (&scratchmat1);

	return;
}



/* Given:
 * z = measurement vector
 * H = observation matrix
 * R = measurement covariance
 * x = state estimate
 * P = state estimate covariance
 * thresh = the error in the normalized z-Hx consistent with the covariance
 * ie, a thresh sigma error is consistent (probably between 3 and 5 )
 *
 * Check to make sure (z-Hx)*(z-Hx)' < (R + H*P*H')*thresh^2
 * If everything is ok, return a 1 else return 0 (for now) */

int meas_consistent(dvec *zp, dmat *Hp, dmat *Rp, dvec *xp, dmat *Pp, float thresh)
{
	int nm, nst, i, cons;
	dmat m1 = ZERODMAT;
	dmat m2 = ZERODMAT;
	dmat m3 = ZERODMAT;
	dmat trans = ZERODMAT;
	dmat dirvar = ZERODMAT;
	dvec v1 = ZERODVEC;
	dvec v2 = ZERODVEC;
	double norm;

	/* Add error checking later */


	nm = zp->neh - zp->nel + 1;
	nst = xp->neh - xp->nel + 1;

	if (nm <= nst)
		return 1;

	/* m1 = H*P */
	dmatxdmat(Hp,Pp,&m1);
	/* m2 = H' */
	transdmat(Hp, &m2);
	/* m3 = m1*m2 = H*P*H' */
	dmatxdmatsim(&m1, &m2, &m3);

	freedmat(&m1);
	/* m1 = m3 + R = H*P*H' + R */
	dmatplsdmat(&m3, Rp, &m1);

	/* v1 = H*x */
	dmatxdvec(Hp, xp, &v1);

	/* v2 = -v1 = -H*x */
	dvecxscal(&v1, -1.0, &v2);

	freedvec(&v1);
	freedmat(&m2);
	/* v1 = z-Hx */
	dvecplsdvec(zp, &v2, &v1);
	freedvec(&v2);
	norm=dvec2norm(&v1);
	initdmat(&trans, 1, 1, 1, v1.neh-v1.nel+1);
	for (i=1;i<=v1.neh-v1.nel+1;i++)
		setdmat(&trans, 1, i, v1.vec[v1.nel+i-1]/norm);

	rotcov(&m1, &trans, &dirvar);
	/*  printdmat(&dirvar);*/

	if (dirvar.mat[1][1]*thresh*thresh < norm*norm)
		cons = 0;
	else
		cons = 1;


	freedmat(&trans);
	freedmat(&dirvar);
	freedmat(&m1);
	freedmat(&m2);
	freedmat(&m3);
	freedvec(&v1);
	freedvec(&v2);

	return cons;

}

void dvec_outer_product(dvec *vecp, dmat *ansp)
{
	int ne, i, j;

	/* Add error checking later */
	if (!dvecinitialized(vecp))
		nrerror("Vec not initialized in dvec_outer_product");

	ne = vecp->neh - vecp->nel + 1;

	if (!dmatinitialized(ansp))
		initdmat(ansp, 1, ne, 1, ne);
	else
		if ((ansp->nrh - ansp->nrl + 1 != ne) || (ansp->nch - ansp->ncl + 1 != ne))
			nrerror("Ans initialized, but wrong dimension in dvec_outer_product");


	for (i=0;i<ne;i++)
		for (j=0;j<ne;j++)
		{
			setdmat(ansp, i+ansp->nrl, j+ansp->ncl, vecp->vec[vecp->nel+i]*vecp->vec[vecp->nel+j]);
			setdmat(ansp, j+ansp->nrl, i+ansp->ncl, vecp->vec[vecp->nel+i]*vecp->vec[vecp->nel+j]);
		}

	return;
}


void Axebsolve(dmat *Ap, dvec *bp, dvec *xp)
{
	dmat Asinv = ZERODMAT;
	dmat AtA = ZERODMAT;
	dmat At = ZERODMAT;
	dvec Atb = ZERODVEC;

	/* Add error checking! */

	hth(Ap, &AtA);
	dmatsinv(&AtA, &Asinv);
	transdmat(Ap, &At);
	dmatxdvec(&At, bp, &Atb);
	dmatxdvec(&Asinv, &Atb, xp);

	freedmat(&Asinv);
	freedmat(&AtA);
	freedmat(&At);
	freedvec(&Atb);

}

void Axebsolve2(dmat *Ap, dvec *bp, dvec *xp)
{
	dmat AtA = ZERODMAT;
	dmat At = ZERODMAT;
	dvec Atb = ZERODVEC;
	int n;

	/* Add error checking! */

	hth(Ap, &AtA);
	n=ncols(&AtA);
	dcholfact(AtA.mat, n);

	transdmat(Ap, &At);
	dmatxdvec(&At, bp, &Atb);
	/*initdvec(xp, 1, n);   Jul 8, 98*/
	dcholbksb(AtA.mat, xp->vec, Atb.vec, n);

	freedmat(&AtA);
	freedmat(&At);
	freedvec(&Atb);

}


void hth(dmat *Hp, dmat *ansp)
{
	int i,j,k,nr, nc, nrl, ncl, nrh, nch, nrlans, nrhans, nclans, nchans, nrans, ncans;

	if (Hp->mat == (double **) 0)
		nrerror("matrix not initialized in hth\n");


	nrl = Hp->nrl;
	nrh = Hp->nrh;
	ncl = Hp->ncl;
	nch = Hp->nch;
	nr = nrh-nrl+1;
	nc = nch-ncl+1;

	/* Create memory for ans, if not already done */
	if (ansp->mat == (double **) 0)
		initdmat(ansp, 1, nc, 1, nc);

	nrlans = ansp->nrl;
	nrhans = ansp->nrh;
	nclans = ansp->ncl;
	nchans = ansp->nch;
	nrans = nrhans-nrlans+1;
	ncans = nchans-nclans+1;

	if ((nrans != nc) || (ncans != nc))
		nrerror("Memory for answer allocated, but wrong dimension in hth");

	for (i=1;i<=nc;i++)
		for (j=i;j<=nc;j++) {
			ansp->mat[nrlans+i-1][nclans+j-1] = 0.0;
			for (k=0;k<=nr-1;k++)
				ansp->mat[nrlans+i-1][nclans+j-1] += Hp->mat[nrl+k][ncl+i-1]*Hp->mat[k+nrl][j+ncl-1];
			ansp->mat[nrlans+j-1][nclans+i-1] = ansp->mat[nrlans+i-1][nclans+j-1];
		}

	return;
}


void hht(dmat *Hp, dmat *HtHp)
{
	dmat Ht = ZERODMAT;
	/* Add error checking! */

	transdmat(Hp, &Ht);
	dmatxdmatsim(Hp, &Ht, HtHp);

	freedmat(&Ht);

	return;
}

void dmat2diag(dmat *matp, dvec *vecp)
{
	int nr, nc, i;

	if (!dmatinitialized(matp))
		nrerror("Matrix not initialized in dmat2diag");

	nr = matp->nrh - matp->nrl + 1;
	nc = matp->nch - matp->ncl + 1;

	if (nr != nc)
		nrerror("Matrix not square in dmat2diag");

	if (!dvecinitialized(vecp))
		initdvec(vecp, 1, nc);
	else
		if (vecp->neh - vecp->nel != nc)
			nrerror("Vector initialized to wrong dimension in dmat2diag");

	for (i=0;i<=nc-1;i++)
		setdvec(vecp, vecp->nel+i, matp->mat[matp->nrl+i][matp->ncl+i]);

	return;
}

/* Rotate covariance from frame x to frame y where y=Rx
   ie, find Py=R*Px*Rt */
void rotcov(dmat *cxp, dmat *Rp, dmat *cyp)
{
	dmat scratch = ZERODMAT;
	dmat scratch2 = ZERODMAT;


	dmatxdmat(Rp, cxp, &scratch);
	transdmat(Rp, &scratch2);
	dmatxdmatsim(&scratch, &scratch2, cyp);

	freedmat(&scratch);
	freedmat(&scratch2);
	return;
}

/* Find At*S*A */
void AtSA(dmat *Sp, dmat *Ap, dmat *ansp)
{
	dmat scratch = ZERODMAT;
	dmat scratch2 = ZERODMAT;

	transdmat(Ap, &scratch2);
	dmatxdmat(&scratch2, Sp, &scratch);
	dmatxdmatsim(&scratch, Ap, ansp);

	freedmat(&scratch);
	freedmat(&scratch2);
	return;
}

void new_get_Att_Rot( dmat *Ap, double azi, double pit, double rol ) {

	int nr, nc;

	if (!dmatinitialized(Ap))
		initdmat(Ap, 1, 3, 1, 3);

	nr = Ap->nrh - Ap->nrl + 1;
	nc = Ap->nch - Ap->ncl + 1;

	if ((nr != 3) || (nc != 3))
		nrerror("Matrix initialized, but wrong size in new_get_Att_Rot");

	setdmat(Ap, 1, 1, cos(pit)*sin(azi));
	setdmat(Ap, 1, 2, cos(rol)*cos(azi)+sin(rol)*sin(pit)*sin(azi));
	setdmat(Ap, 1, 3, -sin(rol)*cos(azi)+cos(rol)*sin(pit)*sin(azi));
	setdmat(Ap, 2, 1, cos(pit)*cos(azi));
	setdmat(Ap, 2, 2, -cos(rol)*sin(azi)+sin(rol)*sin(pit)*cos(azi));
	setdmat(Ap, 2, 3, sin(rol)*sin(azi)+cos(rol)*sin(pit)*cos(azi));
	setdmat(Ap, 3, 1, sin(pit));
	setdmat(Ap, 3, 2, -sin(rol)*cos(pit));
	setdmat(Ap, 3, 3, -cos(rol)*cos(pit));



}


long int long_abs( long int i ) {

	if ( i < ((long int) 0) )
		return -i;
	else
		return i;

}

void pinvdmat(dmat *Ap, dmat *Ainvp)
{
	dmat atai = ZERODMAT;
	dmat at = ZERODMAT;

	transdmat(Ap, &at);
	hthinv(Ap, &atai);
	dmatxdmat(&atai, &at, Ainvp);

	freedmat(&at);
	freedmat(&atai);

	return;
}

void hthinv(dmat *ap, dmat *ataip)
{
	dmat ata = ZERODMAT;

	hth(ap, &ata);
	dmatsinv(&ata, ataip);

	freedmat(&ata);
}

void dvecmindvec(dvec *vec1p, dvec *vec2p, dvec *ansp)
{
	dvec negv2 = ZERODVEC;

	if ((vec1p->neh-vec1p->nel)!=(vec2p->neh-vec2p->nel))
	{
//		printdvec(vec1p);
//		printdvec(vec2p);
		nrerror("Vector dimensions must agree in dvecmindvec\n");
	}


	dvecxscal(vec2p, -1.0, &negv2);
	dvecplsdvec(vec1p, &negv2,ansp);

	freedvec(&negv2);

	return;
}

void blockmat(dmat *Ap, dmat *Bp, dmat *Cp, dmat *Dp, dmat *ansp)
{
	int i, j, nrtop, nrbot, ncleft, ncright;

	if ((!dmatinitialized(Ap))||(!dmatinitialized(Ap))||(!dmatinitialized(Ap))||(!dmatinitialized(Ap)))
		nrerror("Matrix not initialized in blockdmat");

	nrtop = nrows(Ap);
	if (nrtop != nrows(Bp))
		nrerror("Wrong number of rows in blockmat (top)");

	nrbot = nrows(Cp);
	if (nrbot != nrows(Dp))
		nrerror("Wrong number of rows in blockmat (bot)");

	ncleft = ncols(Ap);
	if (ncleft != ncols(Cp))
		nrerror("Wrong number of cols in blockmat (left)");

	ncright = ncols(Bp);
	if (ncright != ncols(Dp))
		nrerror("Wrong number of cols in blockmat (right)");

	if (!dmatinitialized(ansp))
		initdmat(ansp, 1, nrtop+nrbot, 1, ncleft+ncright);
	else
	{
		if (nrows(ansp)!=nrtop+nrbot)
			nrerror("Matrix initialized, but wrong # of rows in blockmat");
		if (ncols(ansp)!=ncleft+ncright)
			nrerror("Matrix initialized, but wrong # of cols in blockmat");
	}

	/* Upper left */
	for (i=1;i<=nrtop;i++)
		for (j=1;j<=ncleft;j++)
			setdmatrel(ansp, i, j, getdmatrel(Ap, i, j));

	/* Upper right */
	for (i=1;i<=nrtop;i++)
		for (j=1;j<=ncright;j++)
			setdmatrel(ansp, i, j+ncleft, getdmatrel(Bp, i, j));

	/* Lower left */
	for (i=1;i<=nrbot;i++)
		for (j=1;j<=ncleft;j++)
			setdmatrel(ansp, i+nrtop, j, getdmatrel(Cp, i, j));

	/* Lower right */
	for (i=1;i<=nrbot;i++)
		for (j=1;j<=ncright;j++)
			setdmatrel(ansp, i+nrtop, j+ncleft, getdmatrel(Dp, i, j));

	return;
}

void setdmatrel(dmat *Ap, int relrow, int relcol, double value)
{

	setdmat(Ap, Ap->nrl-1+relrow, Ap->ncl-1+relcol, value);

	return;
}

double getdmatrel(dmat *Ap, int relrow, int relcol)
{
	if (!dmatinitialized(Ap))
		nrerror("Matrix not initialized in getdmatrel");

	if ((relrow < 1) || (relrow > nrows(Ap)))
		nrerror("Bad row index in getdmatrel");
	if ((relcol < 1) || (relcol > ncols(Ap)))
		nrerror("Bad col index in getdmatrel");


	return (Ap->mat[Ap->nrl - 1+relrow][Ap->ncl - 1+relcol]);

}

double getdmat(dmat *Ap, int row, int col)
{
	if (!dmatinitialized(Ap))
		nrerror("Matrix not initialized in getdmat");

	if ((row < Ap->nrl) || (row > Ap->nrh))
		nrerror("Bad row index in getdmatrel");
	if ((col < Ap->ncl) || (col > Ap->nch))
		nrerror("Bad col index in getdmat");


	return (Ap->mat[row][col]);

}



int nrows(dmat *Ap)
{
	if (!dmatinitialized(Ap))
		nrerror("Matrix not initialized in nrows");

	return (Ap->nrh - Ap->nrl + 1);
}

int ncols(dmat *Ap)
{
	if (!dmatinitialized(Ap))
		nrerror("Matrix not initialized in ncols");

	return (Ap->nch - Ap->ncl + 1);
}

void copydmat(dmat *Ap, dmat *ansp)
{
	int i, j;

	if (!dmatinitialized(Ap))
		nrerror("Matrix not initialized in copydmat\n");

	if (!dmatinitialized(ansp))
		initdmat(ansp, 1, nrows(Ap), 1, ncols(Ap));
	else
		if ((nrows(Ap) != nrows(ansp))||(ncols(Ap) != ncols(ansp)))
			nrerror("Matrix initialized, but wrong size in copydmat");

	for (i=1;i<=nrows(Ap);i++)
		for (j=1;j<=ncols(Ap);j++)
			setdmatrel(ansp, i, j, getdmatrel(Ap, i, j));
	return;

}

/* Note: this returns a vector with the square of the dops */
void dopvec(dmat *Hp, dvec *vecp)
{
	dmat Hth=ZERODMAT;
	dmat hthi=ZERODMAT;

	hth(Hp, &Hth);
	dmatinv(&Hth,&hthi);
	dmat2diag(&hthi, vecp);


	freedmat(&Hth);
	freedmat(&hthi);

}

double pdop(dmat *Hp)
{
	double ans;
	dvec dopsquare = ZERODVEC;
	dopvec(Hp, &dopsquare);
	ans=sqrt(getdvec(&dopsquare, 1)+getdvec(&dopsquare, 2)+getdvec(&dopsquare, 3));

	freedvec(&dopsquare);
	return (ans);


}


/******BSP**********/



void copydvec(dvec *vec1p, dvec *ansp) {

	int i,ne1,nel1,neh1,nelans,nehans,neans;

	if (vec1p->vec == (double *) 0)
		nrerror("Vector not initialized in copydvec\n");

	/* Check to make sure vectors are correct size */
	nel1 = vec1p->nel;
	neh1 = vec1p->neh;
	ne1 = neh1-nel1+1;


	/* Create memory for ans, if not already done */
	if (ansp->vec == (double *) 0)
		initdvec(ansp,1,ne1);

	nelans = ansp->nel;
	nehans = ansp->neh;
	neans = nehans-nelans+1;

	if (neans != ne1)
		nrerror("Memory for answer allocated, but wrong dimension in copydvec\n");

	for (i=1;i<=ne1;i++)
		ansp->vec[nelans+i-1]=vec1p->vec[nel1+i-1];

	return;

}

void copyivec(ivec *vec1p, ivec *ansp) {

	int i,ne1,nel1,neh1,nelans,nehans,neans;

	if (vec1p->vec == (int *) 0)
		nrerror("Vector not initialized in copyivec\n");

	/* Check to make sure vectors are correct size */
	nel1 = vec1p->nel;
	neh1 = vec1p->neh;
	ne1 = neh1-nel1+1;


	/* Create memory for ans, if not already done */
	if (ansp->vec == (int *) 0)
		initivec(ansp,1,ne1);

	nelans = ansp->nel;
	nehans = ansp->neh;
	neans = nehans-nelans+1;

	if (neans != ne1)
		nrerror("Memory for answer allocated, but wrong dimension in copyivec\n");

	for (i=1;i<=ne1;i++)
		setivec(ansp,nelans+i-1,getivec(vec1p,nel1+i-1));

	return;

}

void Sxebsolve(dmat *Sp, dvec *bp, dvec *xp) {

	/* S must be symmetric and positive definite */

	double **SS,*xx,*bb;

	int i,j,nrs,ncs,neb,nex;

	if (!dmatinitialized(Sp))
		nrerror("Matrix not initialized in Sxebsolve\n");
	if (!dvecinitialized(bp))
		nrerror("Vector not initialized in Sxebsolve\n");
	if (!dvecinitialized(xp))
		nrerror("Vector not initialized in Sxebsolve\n");

	nrs=Sp->nrh-Sp->nrl+1;
	ncs=Sp->nch-Sp->ncl+1;
	neb=bp->neh-bp->nel+1;
	nex=xp->neh-xp->nel+1;

	if( (nrs!=ncs) || (nrs!=neb) || (nrs!=nex) )
		nrerror("Matrix/vector size mismatch in Sxebsolve\n");

	SS=dmatrix(1,nrs,1,nrs);
	xx=dvector(1,nrs);
	bb=dvector(1,nrs);

	for (i=1; i<=nrs; i++) {
		for (j=1; j<=nrs; j++) {
			SS[i][j]=getdmatrel(Sp,i,j);
		}
		bb[i]=getdvec(bp,bp->nel+i-1);
	}



	dcholfact(SS,nrs);
	dcholbksb(SS,xx,bb,nrs);

	for (i=1; i<=nrs; i++)
		setdvec(xp,i,xx[i]);

	free_dmatrix(SS,1,nrs,1,nrs);
	free_dvector(xx,1,nrs);
	free_dvector(bb,1,nrs);

}


void SMeBsolve(dmat *Sp, dmat *Bp, dmat *Xp) {

	/* S must be symmetric and positive definite */

	double **SS,*xx,*bb;

	int i,j,nrs,ncs,nrb,ncb,nrx,ncx;

	if (!dmatinitialized(Sp))
		nrerror("Matrix not initialized in SMebsolve\n");
	if (!dmatinitialized(Bp))
		nrerror("Matrix not initialized in SMebsolve\n");
	nrs=nrows(Sp);
	ncs=ncols(Sp);
	nrb=nrows(Bp);
	ncb=ncols(Bp);
	if (!dmatinitialized(Xp))
		initdmat(Xp, 1, nrs, 1, ncb);

	nrx=nrows(Xp);
	ncx=ncols(Xp);

	if( (nrs!=ncs) || (nrs!=nrx) || (nrs!=nrb) || (ncx!=ncb) )
		nrerror("Matrix size mismatch in SMeBsolve\n");

	SS=dmatrix(1,nrs,1,nrs);
	xx=dvector(1,nrs);
	bb=dvector(1,nrs);

	for (i=1; i<=nrs; i++)
		for (j=1; j<=ncs; j++)
			SS[i][j]=getdmatrel(Sp,i,j);


	dcholfact(SS,nrs);

	for (j=1; j<=ncb; j++) {
		for (i=1; i<=nrs; i++)
			bb[i]=getdmatrel(Bp,i,j);
		dcholbksb(SS,xx,bb,nrs);
		for (i=1; i<=nrs; i++)
			setdmat(Xp,i,j,xx[i]);
	}

	free_dmatrix(SS,1,nrs,1,nrs);
	free_dvector(xx,1,nrs);
	free_dvector(bb,1,nrs);

}


void subivec(ivec *ivecp, int el1, int el2, ivec *ansp)
{
	int i,nsub;

	if (!ivecinitialized(ivecp))
		nrerror("Vector not initialized in subivec\n");

	if ((el1 < ivecp->nel) || (el1 > ivecp->neh) ||(el2 < ivecp->nel) || (el2 > ivecp->neh) || (el2 < el1) )
		nrerror("Bad index in subivec\n");

	nsub = el2-el1+1;

	if (!ivecinitialized(ansp))
		initivec(ansp,1,nsub);
	else
		if ((ansp->neh - ansp->nel +1) != nsub)
			nrerror("Vector initialized, but wrong dimension in subivec");

	for (i=1;i<=nsub;i++)
		setivec(ansp, ansp->nel+i-1, ivecp->vec[el1+i-1]);


	return;

}

/******endBSP*******/



/****BSP3****/


/* Initialize lower triangular matrix (indexed from 1 to 'size') */

void initLdmat(Ldmat *Ldmatp, int size) {

	int i;

	double **m;

	/* Make sure not initialized */

	if (Ldmatp->mat != (double **) 0)
		nrerror("Lower triangular matrix already initialized.\n");

	m=(double **) malloc((unsigned) size*sizeof(double *));
	if (m == (double **) 0)
		nrerror("Memory allocation failure in initLdmat\n");
	Ldmatp->mat = m-1;
	for (i=1; i<=size; i++)
		Ldmatp->mat[i] = dvector(1,i);
	Ldmatp->size = size;

}


void freeLdmat(Ldmat *Ldmatp) {

	int i;

	if (Ldmatp->mat != (double **) 0) {

		for (i=1; i<=Ldmatp->size; i++)
			free_dvector(Ldmatp->mat[i],1,i);
		free((char*) (Ldmatp->mat+1));
		Ldmatp->mat = (double **) 0;
		Ldmatp->size = 0;

	}

}


void zeroLdmat(Ldmat *Ldmatp) {

	int i,j;

	if (Ldmatp->mat == (double **) 0)
		nrerror("Matrix not initialized in zeroLdmat\n");

	for (i=1; i<=Ldmatp->size; i++)
		for (j=1; j<=i; j++)
			Ldmatp->mat[i][j]=0.0;

	return;
}


void Ldmat2dmat(Ldmat *Ldmatp, dmat *dmatp) {

	int i,j,ii,jj;

	if ( (dmatp->mat == (double **) 0) || (Ldmatp->mat == (double **) 0) )
		nrerror("Matrix not initialized in Ldmat2dmat.\n");

	if ( ( (dmatp->nrh-dmatp->nrl+1) != (dmatp->nch-dmatp->ncl+1) ) ||
			( (dmatp->nrh-dmatp->nrl+1) != Ldmatp->size) )
		nrerror("Matrix size mismatch in Ldmat2dmat.\n");

	for (i=1; i<=Ldmatp->size; i++) {
		ii=dmatp->nrl+i-1;
		for (j=1; j<i; j++) {
			jj=dmatp->ncl+j-1;
			setdmat(dmatp,ii,jj,Ldmatp->mat[i][j]);
			setdmat(dmatp,jj,ii,Ldmatp->mat[i][j]);
		}
	}
	for (i=1; i<=Ldmatp->size; i++) {
		setdmat(dmatp,i,i,Ldmatp->mat[i][i]);
	}

}


void dmat2Ldmat(dmat *dmatp, Ldmat *Ldmatp) {

	int i,j,ii,jj;

	if ( (dmatp->mat == (double **) 0) || (Ldmatp->mat == (double **) 0) )
		nrerror("Matrix not initialized in dmat2Ldmat.\n");

	if ( ( (dmatp->nrh-dmatp->nrl+1) != (dmatp->nch-dmatp->ncl+1) ) ||
			( (dmatp->nrh-dmatp->nrl+1) != Ldmatp->size) )
		nrerror("Matrix size mismatch in dmat2Ldmat.\n");

	for (i=1; i<=Ldmatp->size; i++) {
		ii=dmatp->nrl+i-1;
		for (j=1; j<=i; j++) {
			jj=dmatp->ncl+j-1;
			Ldmatp->mat[i][j]=getdmat(dmatp,ii,jj);
		}
	}

}

void AS2ASinv(dmat *Ap, dmat *Sp, dmat *ansp)
{
	dmat At = ZERODMAT;
	dmat SinvAt = ZERODMAT;

	transdmat(Ap, &At);
	SMeBsolve(Sp, &At, &SinvAt);

	transdmat(&SinvAt, ansp);

	freedmat(&At);
	freedmat(&SinvAt);

	return;
}

/***BSP9***/

double dmat_inf_norm(dmat *Ap) {

	double in_sav,in;

	int i,j;

	if (Ap->mat == (double **) 0)
		nrerror("Matrix not initialized in dmat_inf_norm\n");

	in_sav=0.0;
	for (i=Ap->nrl; i<=Ap->nrh; i++) {
		in=0.0;
		for (j=Ap->ncl; j<=Ap->nch; j++) {
			in+=fabs(getdmat(Ap,i,j));
		}
		if (in>in_sav) in_sav=in;
	}

	return (in_sav);

}

/***endBSP9***/

void left_null(dmat *Ap, dmat *Lp)
{
	dmat At=ZERODMAT;
	dmat Lt=ZERODMAT;

	transdmat(Ap, &At);

	null(&At, &Lt);
	transdmat(&Lt, Lp);

	freedmat(&At);
	freedmat(&Lt);


}


/* null space of matrix A which is assumed to be full rank
ie- only the null space returned will be dimension n-m */
void null(dmat *Ap, dmat *Lp)
{
	dmat V=ZERODMAT;
	dvec w=ZERODVEC;
	double nulltol = 0.00000001;
	int m, n, i, j, nullity;

	/* Add more error checking */

	m = nrows(Ap);
	n = ncols(Ap);

	if (m>=n)
		nrerror("null() cannot find null space if more rows than columns");

	nullity=n-m;

	dsvdcmp(Ap, &w, &V);

	if (!dmatinitialized(Lp))
		initdmat(Lp, 1, n, 1, nullity);


	for (i=1;i<=n;i++)
		if (getdvec(&w, i) < nulltol)
		{
			for (j=1;j<=n;j++)
				setdmat(Lp, j, nullity, getdmat(&V, j, i));
			nullity--;
		}


	freedmat(&V);
	freedvec(&w);

}


/* Singular Value Decomposition
 Modified from numerical recipes in c routine by dgl 1/16/95 */

void dsvdcmp(dmat *Ap, dvec *wp, dmat *Vp)
{
	int m, n, i, j;
	dmat B = ZERODMAT;

	/* Add more error checking */

	if (!dmatinitialized(Ap))
		nrerror("matrix not initialized in dsvdcmp");


	m=nrows(Ap);
	n=ncols(Ap);

	if (m<n)
	{
		initdmat(&B, 1, n, 1, n);
		zerodmat(&B);
		for (i=1;i<=m;i++)
			for (j=1;j<=n;j++)
				setdmat(&B, i, j, getdmat(Ap, i, j));
		m=n;
	}
	else
		copydmat(Ap, &B);

	if (!dvecinitialized(wp))
		initdvec(wp, 1, n);

	if (!dmatinitialized(Vp))
		initdmat(Vp, 1, n, 1, n);

	svdcmp(B.mat, m, n, wp->vec, Vp->mat);

	freedmat(&B);
}


static double at,bt,ct;
#define PYTHAG(a,b) ((at=fabs(a)) > (bt=fabs(b)) ? \
		(ct=bt/at,at*sqrt(1.0+ct*ct)) : (bt ? (ct=at/bt,bt*sqrt(1.0+ct*ct)): 0.0))

static double maxarg1,maxarg2;
#define MAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?\
		(maxarg1) : (maxarg2))
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

void svdcmp(double **a, int m,int n,double *w,double **v)
{
	int flag,i,its,j,jj,k,l,nm;
	double c,f,h,s,x,y,z;
	double anorm=0.0,g=0.0,scale=0.0;
	double *rv1;

	if (m < n) nrerror("SVDCMP: You must augment A with extra zero rows");
	rv1=dvector(1,n);
	for (i=1;i<=n;i++) {
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) {
			for (k=i;k<=m;k++) scale += fabs(a[k][i]);
			if (scale) {
				for (k=i;k<=m;k++) {
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				if (i != n) {
					for (j=l;j<=n;j++) {
						for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
						f=s/h;
						for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
					}
				}
				for (k=i;k<=m;k++) a[k][i] *= scale;
			}
		}
		w[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m && i != n) {
			for (k=l;k<=n;k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k=l;k<=n;k++) {
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
				if (i != m) {
					for (j=l;j<=m;j++) {
						for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
						for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
					}
				}
				for (k=l;k<=n;k++) a[i][k] *= scale;
			}
		}
		anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}
	for (i=n;i>=1;i--) {
		if (i < n) {
			if (g) {
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}
	for (i=n;i>=1;i--) {
		l=i+1;
		g=w[i];
		if (i < n)
			for (j=l;j<=n;j++) a[i][j]=0.0;
		if (g) {
			g=1.0/g;
			if (i != n) {
				for (j=l;j<=n;j++) {
					for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
					f=(s/a[i][i])*g;
					for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
				}
			}
			for (j=i;j<=m;j++) a[j][i] *= g;
		} else {
			for (j=i;j<=m;j++) a[j][i]=0.0;
		}
		++a[i][i];
	}
	for (k=n;k>=1;k--) {
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=1;l--) {
				nm=l-1;
				if (fabs(rv1[l])+anorm == anorm) {
					flag=0;
					break;
				}
				if (fabs(w[nm])+anorm == anorm) break;
			}
			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];
					if (fabs(f)+anorm != anorm) {
						g=w[i];
						h=PYTHAG(f,g);
						w[i]=h;
						h=1.0/h;
						c=g*h;
						s=(-f*h);
						for (j=1;j<=m;j++) {
							y=a[j][nm];
							z=a[j][i];
							a[j][nm]=y*c+z*s;
							a[j][i]=z*c-y*s;
						}
					}
				}
			}
			z=w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j=1;j<=n;j++) v[j][k]=(-v[j][k]);
				}
				break;
			}
			if (its == 30) nrerror("No convergence in 30 SVDCMP iterations");
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=PYTHAG(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=PYTHAG(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g=g*c-x*s;
				h=y*s;
				y=y*c;
				for (jj=1;jj<=n;jj++) {
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=PYTHAG(f,h);
				w[j]=z;
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=(c*g)+(s*y);
				x=(c*y)-(s*g);
				for (jj=1;jj<=m;jj++) {
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;
			rv1[k]=f;
			w[k]=x;
		}
	}
	free_dvector(rv1,1,n);
}

/* find res'*inv(Wi)*res where Wi is positive definite and symmetric */
double weight_res(dvec *resp, dmat *Wip)
{
	dvec vec = ZERODVEC;
	double wr;

	copydvec(resp, &vec);
	Sxebsolve(Wip, resp, &vec);

	wr=sqrt(dvecdotdvec(resp, &vec));

	freedvec(&vec);

	return wr;

}

void printdmat(dmat *dmatp)
{
	int i,j;

	if (!dmatinitialized(dmatp))
	{
		printf("Matrix not initialized in printdmat\n");
		return;
	}

	printf("\n");

	for (i=dmatp->nrl;i<=dmatp->nrh;i++)
	{
		for (j=dmatp->ncl;j<=dmatp->nch;j++)
			printf("%f ",dmatp->mat[i][j]);
		printf("\n");
	}

	printf("\n");


	return;
}

void printdvec(dvec *dvecp)
{
	int i;

	if (!dvecinitialized(dvecp))
	{
		printf("Vector not initialized in printdvec\n");
		return;
	}

	printf("\n");

	for (i=dvecp->nel;i<=dvecp->neh;i++)
		printf("%f\n",dvecp->vec[i]);

	printf("\n");

	return;
}

void printivec(ivec *dvecp)
{
	int i;

	if (!ivecinitialized(dvecp))
	{
		printf("Vector not initialized in printivec\n");
		return;
	}


	for (i=dvecp->nel;i<=dvecp->neh;i++)
		printf("%d\n",dvecp->vec[i]);



	return;
}
