import itertools
import logging

from sims_solvers.FrontGenerators.FrontGeneratorStrategy import FrontGeneratorStrategy

log = logging.getLogger(__name__)

class AnejaNair(FrontGeneratorStrategy):
    """
    A concrete front generator strategy using the Aneja and Nair iterative dichotomic method.
    It computes the supported efficient solutions for a bi-objective problem by
    repeatedly solving weighted single-objective problems.
    """

    def set_multiply_solution_by_minus_one(self):
        """
        For minimization models (like the set covering problem), no transformation is required.
        For maximization models, we might need to multiply the solution objectives by -1.
        """
        return False if self.solver.model.is_a_minimization_model() else True

    def always_add_new_solutions_to_front(self):
        """
        In the dichotomic method every new solution is a candidate to be part of the Pareto front.
        """
        return True

    @staticmethod
    def compute_weights(z1: list[float], z2: list[float]):
        """
        Given two objective vectors z1 = [a, b] and z2 = [c, d],
        compute a weights vector using the normal vector to the line joining them.
        
        A normal vector can be computed as (d - b, a - c). If any component is negative,
        we flip the signs to guarantee nonnegativity, and then we normalize so that w1 + w2 = 1.
        """
        w1 = z2[1] - z1[1]
        w2 = z1[0] - z2[0]
        if w1 < 0 or w2 < 0:
            w1, w2 = -w1, -w2
        s = w1 + w2
        if s == 0:
            return (0.5, 0.5)
        return (w1 / s, w2 / s)

    
    def solve(self):
        """
        Main method to generate the front of supported efficient solutions
        using an iterative dichotomic procedure.
        
        The method works by:
          1. Obtaining the two extreme solutions using weights (1, 0) and (0, 1).
          2. Maintaining a list of intervals (pairs of consecutive solutions).
          3. For each interval, computing the weight vector (via compute_weight), solving
             the weighted problem, and, if a new and distinct solution is found, inserting it.
          4. Repeating until no more new solutions are found or a timeout is hit
        """

        # Step 1: Solve for the extreme solutions.
        # In the case of 2 objectives, it will return the extreme points of the Pareto front
        log.info("Computing the edge solutions.")
        current_list = list(self.get_best_worst_values())

        # Step 3: Initialize solution set and explored intervals without results.        
        explored_none_intervals = set()

        for solution in current_list:
            yield solution


        # Step 2: Iteratively refine the Pareto front.
        log.info("Iteratively refining the Pareto front.")
        while True:
            # Intermediate list of solutions.
            intermediate_list = []

            for (left, right) in itertools.pairwise(current_list):
                left_obj = left['objs']
                right_obj = right['objs']

                if (tuple(left_obj), tuple(right_obj)) not in explored_none_intervals:

                    weights = self.compute_weights(left_obj, right_obj)
                    log.info(f"Computed weight vector: {weights}")

                    self.solver.set_weighted_sum_objective(weights)
                    new_time = self.get_solver_solution_for_timeout(optimize_not_satisfy=True)
                    new_solution = self.process_feasible_solution(new_time)

                    if new_solution is not None:
                        yield new_solution
                    else:
                        explored_none_intervals.add((tuple(left_obj), tuple(right_obj)))

                    intermediate_list.append(solution)
                else:
                    intermediate_list.append(None)
        
            # If no new elements, stop searching.
            if all(solution is None for solution in intermediate_list):
                break

            # Build the new list of solutions by interleaving the intermediate list with the current list.
            new_list = []
            for current_solution, intermediate_solution in zip(current_list, intermediate_list):
                new_list.append(current_solution)
                if intermediate_solution is not None:
                    new_list.append(intermediate_solution)
            # Add the last element of the current list.
            new_list.append(current_list[-1])

            current_list = new_list


    def get_best_worst_values(self):
        if len(self.solver.model.objectives) == 2:
            all_solutions = self.get_best_worst_for_2obj_lexicographically()
            for formatted_solution in all_solutions:
                if formatted_solution is not None:
                    yield formatted_solution
                else:
                    raise TimeoutError()
