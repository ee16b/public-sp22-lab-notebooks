/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 * 
 * EE 16B Spring 2021
 * Hossein Najafi
 */


#define SIZE                        200
#define SIZE_AFTER_FILTER           1

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and EUCLIDEAN_THRESHOLD
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define EUCLIDEAN_THRESHOLD            0

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = ;
float pca_vec2[SNIPPET_SIZE] = ;
float projected_mean_vec[2] = ;
float centroid1[2] = ; // DRIVE_FAR
float centroid2[2] = ; // DRIVE_LEFT
float centroid3[2] = ; // DRIVE_CLOSE
float centroid4[2] = ; // DRIVE_RIGHT
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


// Initialize your test word here
int16_t result[SNIPPET_SIZE] = {0};



float proj1 = 0;
float proj2 = 0;


/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  
}

void loop(void) {
    // Reset projection result variables declared above
    
    proj1 = 0;
    proj2 = 0;

    /*---------------------------*/
    /*      CODE BLOCK PCA3      */
    /*---------------------------*/

    // Project 'result' onto the principal components
    // Hint: 'result' is an array
    // Hint: the principal components are unit norm
    // Hint: do this entire operation in 1 loop by replacing the '...'
    // YOUR CODE HERE
    for (int i = 0; i < SNIPPET_SIZE; i++) {
      proj1 += ...;
      proj2 += ...;
    }

    // Demean the projection
    proj1 -= ...;
    proj2 -= ...;

    // Classification
    // Use the function 'l2_norm' defined above
    // ith centroid: 'centroids[i]'
    float best_dist = 999999;
    int best_index = -1;
    // YOUR CODE HERE: set 'best_dist' and 'best_index' to appropriate values.
    

    // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
    // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
    // Otherwise, the recording is noise
    // You can use the following printing lines for help
    // YOUR CODE HERE

    /*---------------------------*/
    /*---------------------------*/
    /*---------------------------*/

  delay(2000);
}
