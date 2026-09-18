import numpy as np
import roboticstoolbox as rtb


class FuzzyIKSolver:

    def __init__(self, robot: rtb.Robot):

        self.robot = robot

    # ---------------------------------------------------------
    # FUZZIFICATION
    # ---------------------------------------------------------

    def fuzzify_error(
        self,
        error_val,
        max_err=1.0
    ):

        e = np.clip(
            abs(error_val) / max_err,
            0,
            1
        )

        small = (
            max(0, 1 - 2 * e)
            if e < 0.5
            else 0
        )

        medium = max(
            0,
            1 - abs(2 * e - 1)
        )

        large = (
            max(0, 2 * e - 1)
            if e > 0.5
            else 0
        )

        return (
            small,
            medium,
            large
        )

    # ---------------------------------------------------------
    # FUZZY GAIN
    # ---------------------------------------------------------

    def fuzzy_gain_rules(
        self,
        small,
        medium,
        large
    ):

        out_small = 0.05
        out_medium = 0.2
        out_large = 0.5

        numerator = (
            small * out_small
            + medium * out_medium
            + large * out_large
        )

        denominator = (
            small
            + medium
            + large
        )

        if denominator == 0:

            return out_small

        return numerator / denominator

    # ---------------------------------------------------------
    # IK SOLVER
    # ---------------------------------------------------------

    def solve_ik(
        self,
        target_pose,
        q0,
        max_iter=200,
        tol=1e-3,
        restarts=5
    ):

        # First try Robotics Toolbox LM solver
        try:

            sol = self.robot.ikine_LM(
                target_pose,
                q0=q0,
                joint_limits=True
            )

            if sol.success:

                return sol.q, True

        except Exception as e:

            print(
                "RTB IK failed:",
                e
            )

        # -----------------------------------------------------
        # Fuzzy iterative solver
        # -----------------------------------------------------

        best_q = np.copy(q0)

        min_pos_err = float("inf")

        for attempt in range(
            restarts + 1
        ):

            if attempt == 0:

                q = np.copy(q0)

            else:

                if self.robot.qlim is not None:

                    q = np.random.uniform(
                        self.robot.qlim[0],
                        self.robot.qlim[1]
                    )

                else:

                    q = np.random.uniform(
                        -np.pi,
                        np.pi,
                        self.robot.n
                    )

            for _ in range(max_iter):

                T_current = (
                    self.robot.fkine(q)
                )

                error_vector = np.concatenate(
                    [
                        target_pose.t
                        - T_current.t,

                        target_pose.rpy()
                        - T_current.rpy()
                    ]
                )

                pos_error_mag = (
                    np.linalg.norm(
                        error_vector[:3]
                    )
                )

                ori_error_mag = (
                    np.linalg.norm(
                        error_vector[3:]
                    )
                )

                if pos_error_mag < min_pos_err:

                    min_pos_err = pos_error_mag

                    best_q = np.copy(q)

                if (
                    pos_error_mag < tol
                    and
                    ori_error_mag < tol * 5
                ):

                    return q, True

                # Position fuzzy gain
                s_p, m_p, l_p = (
                    self.fuzzify_error(
                        pos_error_mag,
                        max_err=0.5
                    )
                )

                gain_xyz = (
                    self.fuzzy_gain_rules(
                        s_p,
                        m_p,
                        l_p
                    )
                )

                # Orientation fuzzy gain
                s_o, m_o, l_o = (
                    self.fuzzify_error(
                        ori_error_mag,
                        max_err=1.5
                    )
                )

                base_gain_rpy = (
                    self.fuzzy_gain_rules(
                        s_o,
                        m_o,
                        l_o
                    )
                )

                priority_factor = s_p

                gain_rpy = (
                    base_gain_rpy
                    * priority_factor
                    * 0.5
                )

                W = np.diag(
                    [
                        gain_xyz,
                        gain_xyz,
                        gain_xyz,
                        gain_rpy,
                        gain_rpy,
                        gain_rpy
                    ]
                )

                J = self.robot.jacob0(q)

                damping = 0.05

                A = (
                    J.T
                    @ W
                    @ J
                    + (
                        damping ** 2
                    ) * np.eye(
                        self.robot.n
                    )
                )

                B = (
                    J.T
                    @ W
                )

                try:

                    J_pinv = np.linalg.solve(
                        A,
                        B
                    )

                except np.linalg.LinAlgError:

                    J_pinv = np.linalg.pinv(
                        A
                    ) @ B

                dq = (
                    J_pinv
                    @ error_vector
                )

                q = q + dq

                if self.robot.qlim is not None:

                    q = np.clip(
                        q,
                        self.robot.qlim[0],
                        self.robot.qlim[1]
                    )

        return best_q, False