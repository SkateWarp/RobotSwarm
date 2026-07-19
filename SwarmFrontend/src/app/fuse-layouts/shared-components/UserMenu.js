import Avatar from "@mui/material/Avatar";
import Button from "@mui/material/Button";
import Icon from "@mui/material/Icon";
import ListItemIcon from "@mui/material/ListItemIcon";
import ListItemText from "@mui/material/ListItemText";
import MenuItem from "@mui/material/MenuItem";
import Popover from "@mui/material/Popover";
import Typography from "@mui/material/Typography";
import { useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { Link } from "react-router-dom";
import { logoutUser } from "app/auth/store/userSlice";

function UserMenu() {
    const dispatch = useDispatch();
    const user = useSelector(({ auth }) => auth.user);
    const [userMenu, setUserMenu] = useState(null);
    const firstName = user.firstName || "";
    const lastName = user.lastName || "";
    const initials = `${firstName.charAt(0)}${lastName.charAt(0)}` || "U";
    const isGuest = !user.role || user.role.length === 0;

    const closeMenu = () => setUserMenu(null);

    return (
        <>
            <Button
                aria-label="Abrir menú de usuario"
                className="min-h-40 min-w-40 px-0 md:px-16 py-0 md:py-6"
                color="inherit"
                onClick={(event) => setUserMenu(event.currentTarget)}
            >
                <div className="hidden md:flex flex-col mx-4 items-end">
                    <Typography className="text-11 font-medium" color="textSecondary">
                        {`${firstName} ${lastName}`.trim()}
                    </Typography>
                </div>
                <Avatar className="md:mx-4">{initials}</Avatar>
            </Button>

            <Popover
                anchorEl={userMenu}
                anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
                classes={{ paper: "py-8" }}
                onClose={closeMenu}
                open={Boolean(userMenu)}
                transformOrigin={{ vertical: "top", horizontal: "center" }}
            >
                {isGuest ? (
                    <MenuItem component={Link} onClick={closeMenu} role="button" to="/login">
                        <ListItemIcon className="min-w-40">
                            <Icon>lock</Icon>
                        </ListItemIcon>
                        <ListItemText primary="Iniciar sesión" />
                    </MenuItem>
                ) : (
                    <MenuItem
                        onClick={() => {
                            dispatch(logoutUser());
                            closeMenu();
                        }}
                    >
                        <ListItemIcon className="min-w-40">
                            <Icon>exit_to_app</Icon>
                        </ListItemIcon>
                        <ListItemText primary="Cerrar sesión" />
                    </MenuItem>
                )}
            </Popover>
        </>
    );
}

export default UserMenu;
