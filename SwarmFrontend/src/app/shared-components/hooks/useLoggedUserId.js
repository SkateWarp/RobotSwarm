import jwtDecode from "jwt-decode";

const useLoggedUserId = () => {
    const accessToken = window.localStorage.getItem("jwt_access_token");
    const decoded = jwtDecode(accessToken);

    return decoded.id;
};

export default useLoggedUserId;
