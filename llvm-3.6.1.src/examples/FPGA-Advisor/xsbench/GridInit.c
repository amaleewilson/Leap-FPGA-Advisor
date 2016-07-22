#include "XSbench_header.h"

#ifdef MPI
#include<mpi.h>
#endif

// Generates randomized energy grid for each nuclide
// Note that this is done as part of initialization (serial), so
// rand() is used.
void generate_grids( NuclideGridPoint ** nuclide_grids,
                     long n_isotopes, long n_gridpoints ) {
	for( long i = 0; i < n_isotopes; i++ )
		for( long j = 0; j < n_gridpoints; j++ )
		{
			nuclide_grids[i][j].energy       =((double)rand()/(double)RAND_MAX);
			nuclide_grids[i][j].total_xs     =((double)rand()/(double)RAND_MAX);
			nuclide_grids[i][j].elastic_xs   =((double)rand()/(double)RAND_MAX);
			nuclide_grids[i][j].absorbtion_xs=((double)rand()/(double)RAND_MAX);
			nuclide_grids[i][j].fission_xs   =((double)rand()/(double)RAND_MAX);
			nuclide_grids[i][j].nu_fission_xs=((double)rand()/(double)RAND_MAX);
		}
}

// Verification version of this function (tighter control over RNG)
void generate_grids_v( NuclideGridPoint ** nuclide_grids,
                     long n_isotopes, long n_gridpoints ) {
	for( long i = 0; i < n_isotopes; i++ )
		for( long j = 0; j < n_gridpoints; j++ )
		{
			nuclide_grids[i][j].energy       = rn_v();
			nuclide_grids[i][j].total_xs     = rn_v();
			nuclide_grids[i][j].elastic_xs   = rn_v();
			nuclide_grids[i][j].absorbtion_xs= rn_v();
			nuclide_grids[i][j].fission_xs   = rn_v();
			nuclide_grids[i][j].nu_fission_xs= rn_v();
		}
}


double* opt_energy;

// Sorts the nuclide grids by energy (lowest -> highest)
void sort_nuclide_grids( NuclideGridPoint ** nuclide_grids, long n_isotopes,
                         long n_gridpoints )
{
	int (*cmp) (const void *, const void *);
	opt_energy = (double*) malloc( n_isotopes * n_gridpoints * sizeof( double ) );

	cmp = NGP_compare;

	for( long i = 0; i < n_isotopes; i++ )
	  {
	    qsort( nuclide_grids[i], n_gridpoints, sizeof(NuclideGridPoint), cmp );
	    for( long j = 0; j < n_gridpoints; j++ )
	            opt_energy[i* n_gridpoints + j] = nuclide_grids[i][j].energy;
	  }

	// error debug check
	/*
	for( int i = 0; i < n_isotopes; i++ )
	{
		printf("NUCLIDE %d ==============================\n", i);
		for( int j = 0; j < n_gridpoints; j++ )
			printf("E%d = %lf\n", j, nuclide_grids[i][j].energy);
	}
	*/
}

// Allocates unionized energy grid, and assigns union of energy levels
// from nuclide grids to it.
GridPoint * generate_energy_grid( long n_isotopes, long n_gridpoints,
                                  NuclideGridPoint ** nuclide_grids) {
	int mype = 0;

	#ifdef MPI
	MPI_Comm_rank(MPI_COMM_WORLD, &mype);
	#endif

	if( mype == 0 ) printf("Generating Unionized Energy Grid...\n");


	long n_unionized_grid_points = n_isotopes*n_gridpoints;
	int (*cmp) (const void *, const void *);
	cmp = NGP_compare;

	GridPoint * energy_grid = (GridPoint *)malloc( n_unionized_grid_points
	                                               * sizeof( GridPoint ) );
	if( mype == 0 ) printf("Copying and Sorting all nuclide grids...\n");

	NuclideGridPoint ** n_grid_sorted = gpmatrix( n_isotopes, n_gridpoints );


	memcpy( n_grid_sorted[0], nuclide_grids[0], n_isotopes*n_gridpoints*
	                                      sizeof( NuclideGridPoint ) );

	qsort( &n_grid_sorted[0][0], n_unionized_grid_points,
	       sizeof(NuclideGridPoint), cmp);

	if( mype == 0 ) printf("Assigning energies to unionized grid...\n");

	for( long i = 0; i < n_unionized_grid_points; i++ )
		energy_grid[i].energy = n_grid_sorted[0][i].energy;


	gpmatrix_free(n_grid_sorted);

	int * full = (int *) malloc( n_isotopes * n_unionized_grid_points
	                             * sizeof(int) );
	if( full == NULL )
	{
		fprintf(stderr,"ERROR - Out Of Memory!\n");
		exit(1);
	}

	for( long i = 0; i < n_unionized_grid_points; i++ )
		energy_grid[i].xs_ptrs = &full[n_isotopes * i];

	// debug error checking
	/*
	for( int i = 0; i < n_unionized_grid_points; i++ )
		printf("E%d = %lf\n", i, energy_grid[i].energy);
	*/

	return energy_grid;
}


// Binary Search function for nuclide grid
// Returns ptr to energy less than the quarry that is closest to the quarry
int binary_search_opt( double* A, double quarry, int n )
{
  int min = 0;
  int max = n-1;
  int mid;
  
  // checks to ensure we're not reading off the end of the grid
  if( A[0] > quarry )
    return 0;
  else if( A[n-1] < quarry )
    return n-2;
  
  // Begins binary search
  while( max >= min )
    {
      mid = min + floor( (max-min) / 2.0);
      if( A[mid] < quarry )
	min = mid+1;
      else if( A[mid] > quarry )
	max = mid-1;
      else
	return mid;
    }
  return max;
}

// Searches each nuclide grid for the closest energy level and assigns
// pointer from unionized grid to the correct spot in the nuclide grid.
// This process is time consuming, as the number of binary searches
// required is:  binary searches = n_gridpoints * n_isotopes^2
void set_grid_ptrs( GridPoint * energy_grid, NuclideGridPoint ** nuclide_grids,
                    long n_isotopes, long n_gridpoints )
{
#if 1
  printf("Assigning pointers to Unionized Energy Grid...\n");
  
  for( long j = 0; j < n_isotopes; j++ )
    {
      double* A = &opt_energy[j * n_gridpoints];
      double first = A[0], last = A[n_gridpoints-1];
      for( long i = 0; i < n_isotopes * n_gridpoints ; i++ )
	{
	  double quarry = energy_grid[i].energy;
	  if( first > quarry )
	    energy_grid[i].xs_ptrs[j] = 0;
	  else if( last < quarry )
	     energy_grid[i].xs_ptrs[j] = n_gridpoints - 2;
	  else
	    {
	      int min = 0, mid, max = n_gridpoints-1;
	      while( max >= min )
	  	{
	  	  mid = min + floor( (max-min) / 2.0);
	  	  if( A[mid] < quarry )
	  	    min = mid+1;
	  	  else if ( A[mid] > quarry )
		    max = mid-1;
	  	  else
		    {
		     max = mid;
		     break;
		    }
	  	}
	      energy_grid[i].xs_ptrs[j] = max;
	    }
	  //energy_grid[i].xs_ptrs[j] = binary_search( nuclide_grids[j], quarry, n_gridpoints);
	  //energy_grid[i].xs_ptrs[j] = binary_search_opt( &opt_energy[j * n_gridpoints], quarry, n_gridpoints );
	  //if ( binary_search( nuclide_grids[j], quarry, n_gridpoints) != energy_grid[i].xs_ptrs[j] )
	  //printf("error expected %d actual %d \n", binary_search( nuclide_grids[j], quarry, n_gridpoints), energy_grid[i].xs_ptrs[j] );
	}
    }

#else

	int mype = 0;

	if( mype == 0 ) printf("Assigning pointers to Unionized Energy Grid...\n");
	#pragma omp parallel for default(none) \
	shared( energy_grid, nuclide_grids, n_isotopes, n_gridpoints, mype )
	for( long i = 0; i < n_isotopes * n_gridpoints ; i++ )
	{
		double quarry = energy_grid[i].energy;
		if( INFO && mype == 0 && omp_get_thread_num() == 0 && i % 200 == 0 )
		  printf("\rAligning Unionized Grid...(%.0lf%% complete)",
			 100.0 * (double) i / (n_isotopes*n_gridpoints /
				                         omp_get_num_threads())     );
		for( long j = 0; j < n_isotopes; j++ )
		{
			// j is the nuclide i.d.
			// log n binary search
			energy_grid[i].xs_ptrs[j] =
				binary_search( nuclide_grids[j], quarry, n_gridpoints);
		}
	}
	if( mype == 0 ) printf("\n");
#endif

	//test
	/*
	for( int i=0; i < n_isotopes * n_gridpoints; i++ )
		for( int j = 0; j < n_isotopes; j++ )
			printf("E = %.4lf\tNuclide %d->%p->%.4lf\n",
			       energy_grid[i].energy,
                   j,
				   energy_grid[i].xs_ptrs[j],
				   (energy_grid[i].xs_ptrs[j])->energy
				   );
	*/
}

#if 0 
void set_grid_ptrs( GridPoint * energy_grid, NuclideGridPoint ** nuclide_grids,
                    long n_isotopes, long n_gridpoints )
{	
  for( long i = 0; i < n_isotopes * n_gridpoints ; i++ )
    {
      double quarry = energy_grid[i].energy;
      for( long j = 0; j < n_isotopes; j++ )
	energy_grid[i].xs_ptrs[j] = binary_search( nuclide_grids[j], quarry, n_gridpoints);
    }
}


int binary_search( NuclideGridPoint * A, double quarry, int n )
{
  int min = 0;
  int max = n-1;
  int mid;

  if( A[0].energy > quarry )
    return 0;
  else if( A[n-1].energy < quarry )
    return n-2;

  while( max >= min )
    {
      mid = min + floor( (max-min) / 2.0);
      if( A[mid].energy < quarry )
	min = mid+1;
      else if( A[mid].energy > quarry )
	max = mid-1;
      else
	return mid;
    }
  return max;
}


void set_grid_ptrs( GridPoint * energy_grid, NuclideGridPoint ** nuclide_grids,
                    long n_isotopes, long n_gridpoints )
{
  for( long j = 0; j < n_isotopes; j++ )
    {
      double* A = &opt_energy[j * n_gridpoints];
      double first = A[0], last = A[n_gridpoints-1];
      for( long i = 0; i < n_isotopes * n_gridpoints ; i++ )
	{
	  double quarry = energy_grid[i].energy;
	  if( first > quarry )
	    energy_grid[i].xs_ptrs[j] = 0;
	  else if( last < quarry )
	    energy_grid[i].xs_ptrs[j] = n_gridpoints - 2;
	  else
	    {
	      int min = 0, mid, max = n_gridpoints-1;
	      while( max >= min )
	  	{
	  	  mid = min + floor( (max-min) / 2.0);
	  	  if( A[mid] < quarry )
	  	    min = mid+1;
	  	  else if ( A[mid] > quarry )
		    max = mid-1;
	  	  else
		    {
		      max = mid;
		      break;
		    }
	  	}
	      energy_grid[i].xs_ptrs[j] = max;
	    }
	}
    }
}

#endif
