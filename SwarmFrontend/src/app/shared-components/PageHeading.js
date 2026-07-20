import PropTypes from "prop-types";
import { Box, Stack, Typography } from "@mui/material";

function PageHeading({ title, description, meta, status, actions, children }) {
    return (
        <Box
            component="header"
            sx={{
                mb: 3,
                pb: 2.5,
                borderBottom: 1,
                borderColor: "divider",
            }}
        >
            <Stack
                direction={{ xs: "column", sm: "row" }}
                spacing={2}
                alignItems={{ xs: "stretch", sm: "flex-start" }}
                justifyContent="space-between"
            >
                <Box sx={{ minWidth: 0 }}>
                    <Typography component="h1" variant="h5" sx={{ fontWeight: 600, letterSpacing: "-0.01em" }}>
                        {title}
                    </Typography>
                    {description ? (
                        <Typography color="text.secondary" variant="body2" sx={{ mt: 0.75, maxWidth: 760 }}>
                            {description}
                        </Typography>
                    ) : null}
                    {meta ? (
                        <Typography color="text.secondary" variant="caption" sx={{ display: "block", mt: 1 }}>
                            {meta}
                        </Typography>
                    ) : null}
                </Box>
                {status || actions ? (
                    <Stack
                        direction="row"
                        spacing={1}
                        alignItems="center"
                        justifyContent={{ xs: "flex-start", sm: "flex-end" }}
                        sx={{ flexWrap: "wrap", gap: 1 }}
                    >
                        {status}
                        {actions}
                    </Stack>
                ) : null}
            </Stack>
            {children ? <Box sx={{ mt: 2.5 }}>{children}</Box> : null}
        </Box>
    );
}

PageHeading.propTypes = {
    title: PropTypes.string.isRequired,
    description: PropTypes.node,
    meta: PropTypes.node,
    status: PropTypes.node,
    actions: PropTypes.node,
    children: PropTypes.node,
};

export default PageHeading;
