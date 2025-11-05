import NavLinkAdapter from "@fuse/core/NavLinkAdapter";
import { styled } from "@mui/material/styles";
import { motion } from "framer-motion";
import { useDispatch } from "react-redux";
import { Button, Icon, List, ListItem, ListItemText, Paper } from "@mui/material";
import { openNewAccountDialog } from "./store/accountsSlice";
import settingsConfig from "../../../fuse-configs/settingsConfig";

const StyledListItem = styled(ListItem)(({ theme }) => ({
    color: "inherit!important",
    textDecoration: "none!important",
    height: 40,
    width: "100%",
    borderRadius: 6,
    paddingLeft: 12,
    paddingRight: 12,
    marginBottom: 4,
    "&.active": {
        backgroundColor:
            theme.palette.mode === "light"
                ? "rgba(0, 0, 0, .05)!important"
                : "rgba(255, 255, 255, .1)!important",
        pointerEvents: "none",
        "& .list-item-icon": {
            color: "inherit",
        },
    },
    "& .list-item-icon": {
        fontSize: 16,
        width: 16,
        height: 16,
        marginRight: 16,
    },
}));

function AccountsSidebarContent() {
    const dispatch = useDispatch();

    return (
        <div className="p-0 lg:p-24 lg:ltr:pr-4 lg:rtl:pl-4">
            <Paper
                component={motion.div}
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                className="rounded-0 shadow-none lg:rounded-16 lg:shadow"
            >
                <div className="p-24">
                    <Button
                        variant="contained"
                        color="secondary"
                        className="w-full"
                        onClick={() => dispatch(openNewAccountDialog())}
                    >
                        Crear Usuario
                    </Button>
                </div>
                <List className="pt-0 px-12">
                    <StyledListItem
                        button
                        component={NavLinkAdapter}
                        to="/apps/accounts/all"
                        activeClassName="active"
                    >
                        <Icon className="list-item-icon text-16" color="action">
                            people
                        </Icon>
                        <ListItemText className="truncate" primary="Todos" disableTypography />
                    </StyledListItem>
                    {settingsConfig.layout.project === "GTS" || settingsConfig.layout.project === "GTS-swedish" ? (
                        <StyledListItem
                            button
                            component={NavLinkAdapter}
                            to="/apps/accounts/users"
                            activeClassName="active"
                        >
                            <Icon className="list-item-icon text-16" color="action">
                                person
                            </Icon>
                            <ListItemText className="truncate" primary="Usuarios" disableTypography />
                        </StyledListItem>
                    ) : null}
                    {settingsConfig.layout.project !== "task" &&
                    settingsConfig.layout.project !== "panelTemp" &&
                    settingsConfig.layout.project !== "GTS" &&
                    settingsConfig.layout.project !== "GTS-swedish" ? (
                        <StyledListItem
                            button
                            component={NavLinkAdapter}
                            to="/apps/accounts/operators"
                            activeClassName="active"
                        >
                            <Icon className="list-item-icon text-16" color="action">
                                people
                            </Icon>
                            <ListItemText className="truncate" primary="Operadores" disableTypography />
                        </StyledListItem>
                    ) : null}
                </List>
            </Paper>
        </div>
    );
}
export default AccountsSidebarContent;
