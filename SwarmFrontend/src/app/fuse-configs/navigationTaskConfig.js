import authRoles from '../auth/authRoles';

const navigationTaskConfig = [
  {
    id: 'dashb',
    title: 'Dashboard',
    type: 'item',
    icon: 'dashboard',
    auth: authRoles.user,
    url: '/apps/dashboard/tasks',
  },
  {
    id: 'tasks',
    title: 'Tareas',
    type: 'group',
    icon: 'add_task',
    auth: authRoles.user,
    children: [
      {
        id: 'add_task',
        title: 'Crear',
        type: 'item',
        icon: 'add_circle',
        url: 'apps/tasks/categories'
      },
      {
        id: 'index_task',
        title: 'Tareas',
        type: 'item',
        icon: 'add_task',
        url: '/apps/tasks/index/'
      },
      {
        id: 'assign_task',
        title: 'Asignadas',
        type: 'item',
        icon: 'task',
        url: '/apps/tasks/assign/task'
      },
      {
        id: 'report_task',
        title: 'Reportes',
        type: 'item',
        icon: 'assignment',
        url: '/apps/tasks/report/2'
      },
    ]
  },
  {
    id: 'contacts',
    title: 'Usuarios',
    type: 'item',
    icon: 'contacts',
    auth: authRoles.admin,
    url: '/apps/accounts/accounts',
  },
  {
    id: 'config',
    title: 'Config',
    type: 'group',
    auth: authRoles.admin,
    icon: 'settings',
    children: [
      {
        id: 'task-config',
        title: 'Tareas',
        type: 'item',
        icon: 'add_task',
        url: '/apps/configs/task',
        auth: authRoles.admin
      },
      {
        id: 'category-config',
        title: 'Categorías',
        type: 'item',
        icon: 'add_business',
        url: '/apps/configs/categories',
        auth: authRoles.admin
      },
    ]
  },
];

export default navigationTaskConfig;
