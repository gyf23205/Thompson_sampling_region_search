#include "mex.h"
#include "math.h"
#include <map>

// Create linear index to element in 2D matrix
int sub2ind(int nRows, int rowIdx, int colIdx) {
    return colIdx*nRows + rowIdx;
}

/*Inputs:
 *  0 nodesUnique - list of possible nodes for bots
 *  1 footprint - boolean array of nodes in footprints
 *  2 noDetect - prob of no detection for each node
 *  3 serverPd - prob of detection for server for each node
 *  4 nodes - list of possible configs to test
 *  5 Z - boolean array of possible binary measurements
 *  6 w - weights of particles
 *  7 mu - parameter for false positives
 *  8 id - column corresponding to robot
 *  9 orderApprox - order of log approximation
 *  10 rho - parameter of geometric rate of return
 *  11 tLast - time since last check-in
 *
 *Outputs:
 *  0 goal - node to drive to
 *  1 info - max value of MI
*/

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
    if (nrhs != 12) {
        mexErrMsgTxt("Wrong number of inputs");
    }
    if (nlhs != 2) {
        mexErrMsgTxt("Wrong number of outputs");
    }
    
    // Get inputs /////////////////////////////////////////////////////////
    double  *nodesUnique = mxGetPr(prhs[0]);
    int     nNodes = mxGetM(prhs[0]); // number of unique nodes
    if (mxGetN(prhs[0]) != 1) {
        mexErrMsgTxt("nodesUnique has wrong number of columns");
    }
    
    mxLogical   *footprint = mxGetLogicals(prhs[1]);
    if (mxGetM(prhs[1]) != nNodes) {
        mexErrMsgTxt("footprint has wrong number of rows");
    }
    int     nP = mxGetN(prhs[1]); // number of particles
    
    double  *noDetect = mxGetPr(prhs[2]);
    if (mxGetM(prhs[2]) != nNodes) {
        mexErrMsgTxt("noDetect has wrong number of rows");
    }
    if (mxGetN(prhs[2]) != nP) {
        mexErrMsgTxt("noDetect has wrong number of columns");
    }
    
    double  *serverPd = mxGetPr(prhs[3]);
    if (mxGetM(prhs[3]) != nNodes) {
        mexErrMsgTxt("serverPd has wrong number of rows");
    }
    if (mxGetN(prhs[3]) != 1) {
        mexErrMsgTxt("serverPd has wrong number of columns");
    }
    
    double  *nodes = mxGetPr(prhs[4]);
    int     nConfigs = mxGetM(prhs[4]);
    int     nBots = mxGetN(prhs[4]); // number of robots in group
    
    mxLogical   *Z = mxGetLogicals(prhs[5]);
    if (mxGetM(prhs[5]) != (1 << nBots)) {
        mexErrMsgTxt("Z has wrong number of rows");
    }
    int nZ = 1 << nBots;
    if (mxGetN(prhs[5]) != nBots) {
        mexErrMsgTxt("Z has wrong number of columns");
    }
    
    double  *w = mxGetPr(prhs[6]);
    if (mxGetM(prhs[6]) != 1) {
        mexErrMsgTxt("w has wrong number of rows");
    }
    if (mxGetN(prhs[6]) != nP) {
        mexErrMsgTxt("w has wrong number of columns");
    }
    
    double  mu = mxGetScalar(prhs[7]);
    
    int     id = mxGetScalar(prhs[8]) - 1; // convert to 0-based indexing
    
    int     orderApprox = mxGetScalar(prhs[9]);
    
    double  rho = mxGetScalar(prhs[10]);
    
    int     tLast = mxGetScalar(prhs[11]);
    // find expected number of measurements
    int nMsgs = 0;
    for (int t=0; t<tLast; t++)
        nMsgs += (tLast-t) * pow(1-rho, t) * rho;
    
    // Initialize outputs /////////////////////////////////////////////////
    plhs[0] = mxCreateDoubleScalar(-1);
    double *goal = mxGetPr(plhs[0]);
    
    plhs[1] = mxCreateDoubleScalar(-1000);
    double *maxInfo = mxGetPr(plhs[1]);
    
    // Compute MI /////////////////////////////////////////////////////////
    for (int k=0; k<nConfigs; k++) {
        // check that no collisions
        bool collision = false;
        for (int i=0; i<nBots; i++) {
            for (int j=i+1; j<nBots; j++) {
                if (nodes[sub2ind(nConfigs,k,i)] == nodes[sub2ind(nConfigs,k,j)])
                    collision = true;
            }
        }
        if (collision)
            continue;
        
        bool foot[nP];// = {false}; 
        for (int j=0; j<nP; j++)
            foot[j] = false;
        
        // create coalition footprint
        std::map<int,int> nodeMap;
        for (int i=0; i<nBots; i++) {
            // find index in nodesUnique
            for (int j=0; j<nNodes; j++) {
                if (nodesUnique[j] == nodes[sub2ind(nConfigs,k,i)]) {
                    nodeMap[i] = j;
                    break;
                }
            }
            // find footprint of coalition
            for (int j=0; j<nP; j++)
                foot[j] |= footprint[sub2ind(nNodes,nodeMap[i],j)];
        }
        
        // find size of footprint
        int sizeFoot = 0;
        std::map<int,int> footMap;
        for (int j=0; j<nP; j++) {
            if (foot[j])
                footMap[sizeFoot++] = j;
        }
        
        // compute lambda
        double lambda = 0;
        for (int j=0; j<sizeFoot; j++)
            lambda += w[footMap[j]];
        
        // compute alpha
        double alpha[nZ];
        for (int l=0; l<nZ; l++) {
            alpha[l] = 0;
            for (int j=0; j<sizeFoot; j++) {
                double temp = w[footMap[j]];
                for (int i=0; i<nBots; i++) {
                    if (!Z[i*nZ+l])
                        temp *= noDetect[sub2ind(nNodes,nodeMap[i],footMap[j])];
                }
                alpha[l] += temp;
            }
        }
        
        // compute p(Z)
        double pZ[nZ];
        double pZTotal = 0;
        for (int j=0; j<nZ-1; j++) {
            pZ[j] = 0;
            int C0 = 0; // number of non-detections
            int C1 = 0; // number of detections
            std::map<int,int> zeroMap; // col indices of non-detections
            std::map<int,int> oneMap; // col indices of detections
            for (int i=0; i<nBots; i++) {
                if (!Z[i*nZ+j])
                    zeroMap[C0++] = i;
                else
                    oneMap[C1++] = i;
            }
            // add up to get p(Z)
            for (int l=0; l<nZ; l++) {
                bool match = true;
                // see if C^0 matches
                for (int i=0; i<C0; i++) {
                    if (Z[sub2ind(nZ,l,zeroMap[i])]) {
                        match = false;
                        break;
                    }
                }
                // find C' and compute p(Z)
                if (match) {
                    int Cp = 0;
                    for (int i=0; i<C1; i++) {
                        if (!Z[sub2ind(nZ,l,oneMap[i])])
                            Cp++;
                    }
                    
                    pZ[j] += ((Cp%2)==0 ? 1 : -1) * exp(-(lambda + mu*(C0+Cp) - alpha[l]));
                }
            }
            pZTotal += pZ[j];
            if (pZ[j] < 0 | pZ[j] > 1)
                mexErrMsgTxt("p(Z) not a probability");
        }
        pZ[nZ-1] = 1-pZTotal;
        
        // compute H[Z]
        double Hz = 0;
        for (int i=0; i<nZ; i++)
            Hz -= pZ[i] * log(pZ[i]);
        
        // compute H[Z | X]
        double Hzx = 0;
        for (int i=0; i<nBots; i++) {
            // create footprint map for robot
            footMap.clear();
            sizeFoot = 0;
            for (int j=0; j<nP; j++) {
                if (footprint[sub2ind(nNodes,nodeMap[i],j)])
                    footMap[sizeFoot++] = j;
            }
            // compute lambda and beta
            lambda = 0;
            double beta = 0;
            for (int j=0; j<sizeFoot; j++) {
                lambda += w[footMap[j]];
                beta -= w[footMap[j]] * noDetect[sub2ind(nNodes,nodeMap[i],footMap[j])] * log(noDetect[sub2ind(nNodes,nodeMap[i],footMap[j])]);
            }
            // compute alpha
            double alphaH[orderApprox];
            for (int j=0; j<sizeFoot; j++) {
                double temp = w[footMap[j]];
                for (int l=0; l<orderApprox; l++) {
                    temp *= noDetect[sub2ind(nNodes,nodeMap[i],footMap[j])];
                    alphaH[l] += temp;
                }
            }
            // compute H[Zi | X]
            Hzx += exp(-(lambda + mu - alphaH[0])) * (beta + mu + 1);
            for (int l=1; l<orderApprox; l++)
                Hzx -= exp(-(lambda + mu*(l+1) - alphaH[l])) / ((l+1)*l);
        }
        
        double info = Hz - Hzx;
        // Compute server MI //////////////////////////////////////////////
        if (nBots == 1 && nMsgs > 0) // skip if only one robot
            continue;
        // compute lambda
        lambda = 0;
        for (int j=0; j<nP; j++)
            lambda += w[j];
        // find p(z = 0)
        double pd = serverPd[nodeMap[id]];
        double p0 = exp(-lambda * pd - mu);
        // compute H[Z]
        Hz = -p0*log(p0) - (1-p0)*log(1-p0);
        
        // find beta
        double beta = -lambda * (1-pd) * log(1-pd);
        // compute H[Z | X]
        Hzx = exp(-lambda*pd - mu) * (beta + mu + 1);
        for (int l=1; l<orderApprox; l++) {
            Hzx -= exp(-(lambda*(1 - pow(1-pd,l+1)) + mu*(l+1))) / ((l+1)*l);
        }
        
        info += (nBots-1) * nMsgs * (Hz - Hzx);
        // save goal if best config
        if (info > *maxInfo) {
            *maxInfo = info;
            *goal = k+1;
        }
    }
}
