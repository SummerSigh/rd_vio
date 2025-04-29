#include <rdvio/geometry/essential.h>

/* #define DECOMPOSE_ESSENTIAL_HORN 1 */

namespace rdvio {

namespace {

class Polynomial {
  public:
    // clang-format off
        enum GRevLexMonomials {
            XXX = 0, XXY = 1, XYY = 2, YYY = 3, XXZ = 4, XYZ = 5, YYZ = 6, XZZ = 7, YZZ = 8, ZZZ = 9,
            XX = 10, XY = 11, YY = 12, XZ = 13, YZ = 14, ZZ = 15, X = 16, Y = 17, Z = 18, I = 19
        };
    // clang-format on

    vector<20> v;

    Polynomial(const vector<20> &coefficients) : v(coefficients) {}

  public:
    Polynomial() : Polynomial(vector<20>::Zero()) {}

    Polynomial(double w) {
        v.setZero();
        v[I] = w;
    }

    void set_xyzw(double x, double y, double z, double w) {
        v.setZero();
        v[X] = x;
        v[Y] = y;
        v[Z] = z;
        v[I] = w;
    }

    Polynomial operator-() const { return Polynomial(-v); }

    Polynomial operator+(const Polynomial &b) const {
        return Polynomial(v + b.v);
    }

    Polynomial operator-(const Polynomial &b) const {
        return Polynomial(v - b.v);
    }

    Polynomial operator*(const Polynomial &b) const {
        Polynomial r;
        
        // Cache common multiplications to avoid redundant calculations
        const double vI_bI = v[I] * b.v[I];
        const double vI_bX = v[I] * b.v[X];
        const double vI_bY = v[I] * b.v[Y];
        const double vI_bZ = v[I] * b.v[Z];
        const double vX_bI = v[X] * b.v[I];
        const double vY_bI = v[Y] * b.v[I];
        const double vZ_bI = v[Z] * b.v[I];
        
        // Degree 0 term
        r.v[I] = vI_bI;
        
        // Degree 1 terms - vectorized
        r.v.segment<3>(X) = v[I] * b.v.segment<3>(X) + v.segment<3>(X) * b.v[I];
        
        // Degree 2 terms
        // Cache common products for degree 2
        const double vX_bX = v[X] * b.v[X];
        const double vX_bY = v[X] * b.v[Y];
        const double vX_bZ = v[X] * b.v[Z];
        const double vY_bX = v[Y] * b.v[X];
        const double vY_bY = v[Y] * b.v[Y];
        const double vY_bZ = v[Y] * b.v[Z];
        const double vZ_bX = v[Z] * b.v[X];
        const double vZ_bY = v[Z] * b.v[Y];
        const double vZ_bZ = v[Z] * b.v[Z];
        
        r.v[XX] = vI_bI * b.v[XX] + vX_bX + v[XX] * b.v[I];
        r.v[XY] = vI_bI * b.v[XY] + vX_bY + vY_bX + v[XY] * b.v[I];
        r.v[YY] = vI_bI * b.v[YY] + vY_bY + v[YY] * b.v[I];
        r.v[XZ] = vI_bI * b.v[XZ] + vX_bZ + vZ_bX + v[XZ] * b.v[I];
        r.v[YZ] = vI_bI * b.v[YZ] + vY_bZ + vZ_bY + v[YZ] * b.v[I];
        r.v[ZZ] = vI_bI * b.v[ZZ] + vZ_bZ + v[ZZ] * b.v[I];
        
        // Degree 3 terms
        // Cache products for degree 3
        const double vXX_bI = v[XX] * b.v[I];
        const double vXY_bI = v[XY] * b.v[I];
        const double vXZ_bI = v[XZ] * b.v[I];
        const double vYY_bI = v[YY] * b.v[I];
        const double vYZ_bI = v[YZ] * b.v[I];
        const double vZZ_bI = v[ZZ] * b.v[I];
        
        const double vI_bXX = v[I] * b.v[XX];
        const double vI_bXY = v[I] * b.v[XY];
        const double vI_bXZ = v[I] * b.v[XZ];
        const double vI_bYY = v[I] * b.v[YY];
        const double vI_bYZ = v[I] * b.v[YZ];
        const double vI_bZZ = v[I] * b.v[ZZ];
        
        r.v[XXX] = v[I] * b.v[XXX] + v[X] * b.v[XX] + v[XX] * b.v[X] + v[XXX] * b.v[I];
        r.v[XXY] = v[I] * b.v[XXY] + v[Y] * b.v[XX] + v[X] * b.v[XY] + v[XY] * b.v[X] + v[XX] * b.v[Y] + v[XXY] * b.v[I];
        r.v[XYY] = v[I] * b.v[XYY] + v[Y] * b.v[XY] + v[X] * b.v[YY] + v[YY] * b.v[X] + v[XY] * b.v[Y] + v[XYY] * b.v[I];
        r.v[YYY] = v[I] * b.v[YYY] + v[Y] * b.v[YY] + v[YY] * b.v[Y] + v[YYY] * b.v[I];
        r.v[XXZ] = v[I] * b.v[XXZ] + v[Z] * b.v[XX] + v[X] * b.v[XZ] + v[XZ] * b.v[X] + v[XX] * b.v[Z] + v[XXZ] * b.v[I];
        r.v[XYZ] = v[I] * b.v[XYZ] + v[Z] * b.v[XY] + v[Y] * b.v[XZ] + v[X] * b.v[YZ] + v[YZ] * b.v[X] + v[XZ] * b.v[Y] + v[XY] * b.v[Z] + v[XYZ] * b.v[I];
        r.v[YYZ] = v[I] * b.v[YYZ] + v[Z] * b.v[YY] + v[Y] * b.v[YZ] + v[YZ] * b.v[Y] + v[YY] * b.v[Z] + v[YYZ] * b.v[I];
        r.v[XZZ] = v[I] * b.v[XZZ] + v[Z] * b.v[XZ] + v[X] * b.v[ZZ] + v[ZZ] * b.v[X] + v[XZ] * b.v[Z] + v[XZZ] * b.v[I];
        r.v[YZZ] = v[I] * b.v[YZZ] + v[Z] * b.v[YZ] + v[Y] * b.v[ZZ] + v[ZZ] * b.v[Y] + v[YZ] * b.v[Z] + v[YZZ] * b.v[I];
        r.v[ZZZ] = v[I] * b.v[ZZZ] + v[Z] * b.v[ZZ] + v[ZZ] * b.v[Z] + v[ZZZ] * b.v[I];

        return r;
    }

    const vector<20> &coefficients() const { return v; }
};

Polynomial operator*(const double &scale, const Polynomial &poly) {
    return Polynomial(scale * poly.coefficients());
}

inline matrix<3> to_matrix(const vector<9> &vec) {
    return (matrix<3>() << vec.segment<3>(0), vec.segment<3>(3),
            vec.segment<3>(6))
        .finished();
}

inline matrix<9, 4>
generate_nullspace_basis(const std::array<vector<2>, 5> &points1,
                         const std::array<vector<2>, 5> &points2) {
    // Pre-allocate matrix A
    matrix<5, 9> A;
    
    // Pre-compute homogeneous coordinates for all points
    std::array<vector<3>, 5> homo_points1, homo_points2;
    for (size_t i = 0; i < 5; ++i) {
        homo_points1[i] = points1[i].homogeneous();
        homo_points2[i] = points2[i].homogeneous();
    }
    
    // Fill matrix A more efficiently
    for (size_t i = 0; i < 5; ++i) {
        // Compute outer product directly without creating intermediate matrix
        const vector<3>& p1 = homo_points1[i];
        const vector<3>& p2 = homo_points2[i];
        
        // Fill the 9 elements directly (3x3 blocks)
        for (size_t j = 0; j < 3; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                A(i, j * 3 + k) = p1[j] * p2[k];
            }
        }
    }
    
    // Use JacobiSVD instead of BDCSVD for fixed-size matrices
    Eigen::JacobiSVD<matrix<5, 9>> svd(A, Eigen::ComputeFullV);
    return svd.matrixV().block<9, 4>(0, 5);
}

inline matrix<10, 20> generate_polynomials(const matrix<9, 4> &basis) {
    typedef matrix<3, 3, false, Polynomial> matrix_poly;
    matrix<3> Ex = to_matrix(basis.col(0));
    matrix<3> Ey = to_matrix(basis.col(1));
    matrix<3> Ez = to_matrix(basis.col(2));
    matrix<3> Ew = to_matrix(basis.col(3));

    matrix_poly Epoly;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            Epoly(i, j).set_xyzw(Ex(i, j), Ey(i, j), Ez(i, j), Ew(i, j));
        }
    }

    matrix<10, 20> polynomials;

    matrix_poly EEt = Epoly * Epoly.transpose();
    matrix_poly singular_value_constraints =
        (EEt * Epoly) - (0.5 * EEt.trace()) * Epoly;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            polynomials.row(i * 3 + j) =
                singular_value_constraints(i, j).coefficients();
        }
    }

    Polynomial detE = Epoly.determinant();
    polynomials.row(9) = detE.coefficients();

    return polynomials;
}

inline matrix<10> generate_action_matrix(matrix<10, 20> &polynomials) {
    // Initialize permutation array
    std::array<size_t, 10> perm;
    for (size_t i = 0; i < 10; ++i) {
        perm[i] = i;
    }
    
    // Forward elimination with partial pivoting
    for (size_t i = 0; i < 10; ++i) {
        // Find the pivot with maximum absolute value
        double max_val = 0.0;
        size_t max_idx = i;
        for (size_t j = i; j < 10; ++j) {
            double abs_val = std::abs(polynomials(perm[j], i));
            if (abs_val > max_val) {
                max_val = abs_val;
                max_idx = j;
            }
        }
        
        // Swap rows if needed
        if (max_idx != i) {
            std::swap(perm[i], perm[max_idx]);
        }
        
        // Skip if pivot is zero (singular matrix)
        if (max_val < 1.0e-12) {
            continue;
        }
        
        // Normalize the pivot row
        double pivot = polynomials(perm[i], i);
        polynomials.row(perm[i]) /= pivot;
        
        // Eliminate below
        for (size_t j = i + 1; j < 10; ++j) {
            double factor = polynomials(perm[j], i);
            if (std::abs(factor) > 1.0e-12) {
                polynomials.row(perm[j]) -= factor * polynomials.row(perm[i]);
            }
        }
    }
    
    // Back substitution
    for (size_t i = 9; i > 0; --i) {
        for (size_t j = 0; j < i; ++j) {
            double factor = polynomials(perm[j], i);
            if (std::abs(factor) > 1.0e-12) {
                polynomials.row(perm[j]) -= factor * polynomials.row(perm[i]);
            }
        }
    }

    // Construct action matrix
    matrix<10> action;
    
    // Cache the XX offset for better readability
    const int XX_offset = Polynomial::XX;
    
    // Extract rows for the action matrix
    action.row(0) = -polynomials.block<1, 10>(perm[Polynomial::XXX], XX_offset);
    action.row(1) = -polynomials.block<1, 10>(perm[Polynomial::XXY], XX_offset);
    action.row(2) = -polynomials.block<1, 10>(perm[Polynomial::XYY], XX_offset);
    action.row(3) = -polynomials.block<1, 10>(perm[Polynomial::XXZ], XX_offset);
    action.row(4) = -polynomials.block<1, 10>(perm[Polynomial::XYZ], XX_offset);
    action.row(5) = -polynomials.block<1, 10>(perm[Polynomial::XZZ], XX_offset);
    
    // Set unit vectors for the remaining rows
    action.row(6) = vector<10>::Unit(Polynomial::XX - XX_offset).transpose();
    action.row(7) = vector<10>::Unit(Polynomial::XY - XX_offset).transpose();
    action.row(8) = vector<10>::Unit(Polynomial::XZ - XX_offset).transpose();
    action.row(9) = vector<10>::Unit(Polynomial::X - XX_offset).transpose();

    return action;
}

inline std::vector<vector<3>> solve_grobner_system(const matrix<10> &action) {
    // Pre-allocate memory for results to avoid reallocations
    std::vector<vector<3>> results;
    results.reserve(10); // Maximum possible number of solutions
    
    // Compute eigenvalues and eigenvectors
    Eigen::EigenSolver<matrix<10>> eigen(action, true);
    const auto& eigenvalues = eigen.eigenvalues();
    const auto& eigenvectors = eigen.eigenvectors();
    
    // Cache indices for faster access
    const int X_idx = Polynomial::X - Polynomial::XX;
    const int Y_idx = Polynomial::Y - Polynomial::XX;
    const int Z_idx = Polynomial::Z - Polynomial::XX;
    const int I_idx = Polynomial::I - Polynomial::XX;
    
    // Process eigenvalues and eigenvectors
    for (size_t i = 0; i < 10; ++i) {
        // Check if eigenvalue is real (imaginary part is close to zero)
        if (std::abs(eigenvalues[i].imag()) < 1.0e-10) {
            // Extract real part of eigenvector
            const auto& ev = eigenvectors.col(i).real();
            
            // Get homogeneous coordinates
            const double w = ev(I_idx);
            
            // Avoid division by very small values
            if (std::abs(w) > 1.0e-10) {
                const double inv_w = 1.0 / w; // Compute inverse once
                results.emplace_back(ev(X_idx) * inv_w, ev(Y_idx) * inv_w, ev(Z_idx) * inv_w);
            }
        }
    }
    
    return results;
}

} // namespace

void decompose_essential(const matrix<3> &E, matrix<3> &R1, matrix<3> &R2,
                         vector<3> &T) {
#ifdef DECOMPOSE_ESSENTIAL_HORN
    matrix<3> EET = E * E.transpose();
    double halfTrace = 0.5 * EET.trace();
    vector<3> b;

    vector<3> e0e1 = E.col(0).cross(E.col(1));
    vector<3> e1e2 = E.col(1).cross(E.col(2));
    vector<3> e2e0 = E.col(2).cross(E.col(0));

#if DECOMPOSE_ESSENTIAL_HORN == 1
    if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
        b = e0e1.normalized() * sqrt(halfTrace);
    } else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
        b = e1e2.normalized() * sqrt(halfTrace);
    } else {
        b = e2e0.normalized() * sqrt(halfTrace);
    }
#else
    // Compute bbT more efficiently
    matrix<3> bbT = halfTrace * matrix<3>::Identity() - EET;
    vector<3> bbT_diag = bbT.diagonal();
    
    // Fix typo in variable name (bbt_diag -> bbT_diag)
    // Use direct comparison to find maximum diagonal element
    int max_idx = 0;
    if (bbT_diag(1) > bbT_diag(max_idx)) max_idx = 1;
    if (bbT_diag(2) > bbT_diag(max_idx)) max_idx = 2;
    
    // Compute square root once
    double sqrt_diag = std::sqrt(bbT_diag(max_idx));
    b = bbT.row(max_idx) / sqrt_diag;
#endif

    matrix<3> cofactorsT;
    cofactorsT.col(0) = e1e2;
    cofactorsT.col(1) = e2e0;
    cofactorsT.col(2) = e0e1;

    matrix<3> skew_b;
    skew_b << 0, -b.z(), b.y(), b.z(), 0, -b.x(), -b.y(), b.x(), 0;
    matrix<3> bxE = skew_b * E;

    double bTb = b.dot(b);
    R1 = (cofactorsT - bxE) / bTb;
    R2 = (cofactorsT + bxE) / bTb;
    T = b;
#else
    Eigen::JacobiSVD<matrix<3>> svd(E,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix<3> U = svd.matrixU();
    matrix<3> VT = svd.matrixV().transpose();
    if (U.determinant() < 0) {
        U = -U;
    }
    if (VT.determinant() < 0) {
        VT = -VT;
    }
    matrix<3> W;
    W << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    R1 = U * W * VT;
    R2 = U * W.transpose() * VT;
    T = U.col(2);
#endif
}

std::vector<matrix<3>>
solve_essential_5pt(const std::array<vector<2>, 5> &points1,
                    const std::array<vector<2>, 5> &points2) {
    // Generate nullspace basis
    matrix<9, 4> basis = generate_nullspace_basis(points1, points2);
    
    // Generate polynomial constraints
    matrix<10, 20> polynomials = generate_polynomials(basis);
    
    // Generate action matrix for Gröbner basis method
    matrix<10> action = generate_action_matrix(polynomials);
    
    // Solve the polynomial system
    std::vector<vector<3>> solutions = solve_grobner_system(action);
    
    // Pre-allocate results with exact size
    const size_t num_solutions = solutions.size();
    std::vector<matrix<3>> results;
    results.reserve(num_solutions);
    
    // Convert solutions to essential matrices
    for (const auto& solution : solutions) {
        // Compute homogeneous coordinates once
        vector<4> homogeneous = solution.homogeneous();
        
        // Compute the essential matrix
        results.push_back(to_matrix(basis * homogeneous));
    }
    
    return results;
}

} // namespace rdvio
