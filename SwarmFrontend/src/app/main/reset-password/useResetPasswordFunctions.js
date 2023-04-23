import React from 'react';
import {makeStyles} from "@mui/styles";
import {darken} from "@mui/material/styles";
import * as yup from "yup";
import {useDispatch} from "react-redux";
import {useLocation, useNavigate} from "react-router-dom";
import {useForm} from "react-hook-form";
import {yupResolver} from "@hookform/resolvers/yup/dist/yup";
import * as Actions from "../../store/fuse/messageSlice";
import jwtService from "../../services/jwtService";

const useStyles = makeStyles(theme => ({
    root: {
        background: `radial-gradient(${darken(theme.palette.primary.dark, 0.5)} 0%, ${theme.palette.primary.dark} 80%)`,
        color: theme.palette.primary.contrastText
    }
}));


const schema = yup.object().shape({

    password: yup.string().required('Por favor, ingrese su contraseña.').min(8, 'El mínimo de caracteres debe ser 8.')
        .max(16, 'El máximo de caracteres debe ser 16.')
        .matches(
            /^(?=.*[a-z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,16}$/,
            'Debe tener al menos 1 letra, 1 número y 1 símbolo !@#$%^&*'
        ),
    confirmPassword:
        yup.string().required('Debe confirmar su contraseña.')
            .oneOf([yup.ref('password'), null], 'Las contraseñas no son iguales.'),
});


const defaultValues = {

    password: '',
    confirmPassword: '',
};


const useResetPasswordFunctions = () => {

    const dispatch = useDispatch();
    const classes = useStyles();
    const navigate = useNavigate();
    const queryToken = useQuery().get('token');

    const {control, handleSubmit, formState:{errors}, watch} = useForm({

        mode: 'onChange',
        defaultValues,
        resolver: yupResolver(schema),
    });

    const password = watch('password');
    const confirmPassword = watch('confirmPassword');


    function checkForValidPassword(password) {

        return /[a-z]/.test(password) && /[0-9]/.test(password) &&
            /(?=.*[!@#$%^&*])/.test(password) && password.length >=8 && password.length <=16;
    }


    function canBeSubmitted() {

        return (checkForValidPassword(password) && confirmPassword === password);
    }


    function useQuery() {

        return new URLSearchParams(useLocation().search);
    }


    function onSubmit(data) {

        dispatch(Actions.showMessage({message: 'Cambiando contraseña...'}));

        jwtService.createResetPassword({token: queryToken, ...data}).then(res => {

            dispatch(Actions.showMessage({message: res.data.message}));
        }).catch(() => {

            dispatch(Actions.showMessage({message: 'Fallo en la verificación del token'}));
        }).finally(() => {

            navigate({pathname: '/login'});
        });
    }


    return {classes, control, errors, canBeSubmitted, handleSubmit, onSubmit};
};

export default useResetPasswordFunctions;
