import reducer, { clearLoginError, loginError, loginStarted, loginSuccess } from "./loginSlice";

jest.mock("../../services/jwtService", () => ({
    __esModule: true,
    default: { signInWithEmailAndPassword: jest.fn() },
}));
jest.mock("./userSlice", () => ({
    setUserData: (payload) => ({ type: "test/setUser", payload }),
}));
jest.mock("../../store/fuse/messageSlice", () => ({
    showMessage: (payload) => ({ type: "test/showMessage", payload }),
}));

describe("loginSlice", () => {
    it("tracks the busy state and clears a previous error while a login is running", () => {
        const failed = reducer(undefined, loginError({ message: "Credenciales incorrectas." }));
        const pending = reducer(failed, loginStarted());

        expect(pending.busy).toBe(true);
        expect(pending.error).toBeNull();
    });

    it("leaves the form ready after success or after clearing an error", () => {
        const successful = reducer(undefined, loginSuccess());
        expect(successful).toMatchObject({
            busy: false,
            success: true,
            error: null,
        });

        const cleared = reducer(
            { busy: false, success: false, error: { message: "Error" } },
            clearLoginError()
        );
        expect(cleared.error).toBeNull();
    });
});
